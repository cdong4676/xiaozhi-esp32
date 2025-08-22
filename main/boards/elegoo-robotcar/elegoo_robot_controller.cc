#include "elegoo_robot_controller.h"
#include <cstring>
#include <arpa/inet.h>
#include <esp_timer.h>
#include "application.h"
#include "protocol.h"
#include "audio/audio_service.h"

ElegooRobotController::ElegooRobotController() 
    : uart_port_(UART_NUM_MAX), buffer_size_(0), initialized_(false),
      tcp_socket_fd_(-1), tcp_client_fd_(-1), tcp_port_(0), udp_port_(0), 
      network_running_(false), udp_running_(false),
      uart_task_handle_(nullptr), uart_task_running_(false),
      tcp_task_handle_(nullptr), udp_task_handle_(nullptr),
      camera_manager_(nullptr), video_streaming_(false), video_stream_task_handle_(nullptr),
      network_send_queue_(nullptr), network_send_task_handle_(nullptr), network_send_task_running_(false) {
    // 创建网络发送队列
    network_send_queue_ = xQueueCreate(10, sizeof(NetworkMessage));
    if (network_send_queue_ == nullptr) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "创建网络发送队列失败");
    } else {
        ESP_LOGD(ROBOT_CONTROLLER_TAG, "网络发送队列创建成功");
    }
}

ElegooRobotController::~ElegooRobotController() {
    Shutdown();
    
    // 删除网络发送队列
    if (network_send_queue_ != nullptr) {
        // 清空队列中剩余的消息
        NetworkMessage msg;
        while (xQueueReceive(network_send_queue_, &msg, 0) == pdTRUE) {
            if (msg.data != nullptr) {
                free(msg.data);
            }
        }
        vQueueDelete(network_send_queue_);
        network_send_queue_ = nullptr;
        ESP_LOGD(ROBOT_CONTROLLER_TAG, "网络发送队列已删除");
    }
}

bool ElegooRobotController::Initialize(uart_port_t uart_port, int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin, int buffer_size) {
    if (initialized_) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "控制器已经初始化");
        return true;
    }

    uart_port_ = uart_port;
    buffer_size_ = buffer_size;

    if (!ConfigureUart(baud_rate, tx_pin, rx_pin, rts_pin, cts_pin) || !StartUartTask()) {
        uart_driver_delete(uart_port_);
        return false;
    }

    initialized_ = true;
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "机器人控制器初始化成功，UART端口: %d, 波特率: %d", uart_port_, baud_rate);
    return true;
}

void ElegooRobotController::Shutdown() {
    if (!initialized_) return;

    StopVideoStream();
    StopNetworkSendTask();
    StopNetworkServer();
    
    if (uart_task_running_) {
        uart_task_running_ = false;
        if (uart_task_handle_) {
            vTaskDelay(NETWORK_CLIENT_DISCONNECT_DELAY_MS / portTICK_PERIOD_MS);
            uart_task_handle_ = nullptr;
        }
    }

    uart_driver_delete(uart_port_);
    initialized_ = false;
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "机器人控制器已关闭");
}

void ElegooRobotController::UartReceiveTaskWrapper(void* parameter) {
    static_cast<ElegooRobotController*>(parameter)->UartReceiveTask();
}

void ElegooRobotController::UartReceiveTask() {
    uint8_t data[256];
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UART接收任务开始运行");

    while (uart_task_running_) {
        int len = uart_read_bytes(uart_port_, data, sizeof(data) - 1, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
        
        if (len > 0) {
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "接收到串口数据，长度: %d bytes", len);
            SendNetworkMessage(data, len);
        }
        
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UART接收任务结束");
    vTaskDelete(NULL);
}

bool ElegooRobotController::StartNetworkServer(const char* ip, uint16_t tcp_port, uint16_t udp_port) {
    if (network_running_) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "网络服务器已经运行");
        return true;
    }

    tcp_port_ = tcp_port;
    udp_port_ = udp_port;

    // 创建TCP服务器
    tcp_socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (tcp_socket_fd_ < 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "TCP socket创建失败");
        return false;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(tcp_port_);
    inet_pton(AF_INET, ip, &server_addr.sin_addr.s_addr);

    int opt = 1;
    setsockopt(tcp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(tcp_socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) != 0 ||
        listen(tcp_socket_fd_, 1) != 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "TCP socket配置失败");
        close(tcp_socket_fd_);
        tcp_socket_fd_ = -1;
        return false;
    }

    // 启动网络任务
    network_running_ = true;
    
    // 启动网络发送任务
    StartNetworkSendTask();
    
    if (xTaskCreate(TcpServerTaskWrapper, "tcp_server_task", NETWORK_TASK_STACK_SIZE, 
                   this, NETWORK_TASK_PRIORITY, &tcp_task_handle_) != pdPASS) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "TCP服务器任务创建失败");
        StopNetworkServer();
        return false;
    }

    StartUdpBroadcastTask();
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络服务器启动成功，TCP端口: %d, UDP端口: %d", tcp_port_, udp_port_);
    return true;
}

void ElegooRobotController::StopNetworkServer() {
    if (!network_running_) return;

    network_running_ = false;
    StopNetworkSendTask();
    StopUdpBroadcastTask();

    if (tcp_client_fd_ >= 0) {
        close(tcp_client_fd_);
        tcp_client_fd_ = -1;
    }

    if (tcp_socket_fd_ >= 0) {
        close(tcp_socket_fd_);
        tcp_socket_fd_ = -1;
    }

    if (tcp_task_handle_) {
        vTaskDelay(NETWORK_CLIENT_RECONNECT_DELAY_MS / portTICK_PERIOD_MS);
        tcp_task_handle_ = nullptr;
    }

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络服务器已停止");
}


void ElegooRobotController::SendNetworkMessage(const uint8_t* data, size_t length) {
    if (!data || length == 0) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "无效的字节数据或长度为0");
        return;
    }

    if (network_send_queue_ == nullptr) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "网络发送队列未初始化");
        return;
    }

    // 直接使用外部PSRAM分配内存
    uint8_t* data_copy = (uint8_t*)heap_caps_malloc(length, MALLOC_CAP_SPIRAM);
    if (data_copy == nullptr) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "分配PSRAM内存失败");
        return;
    }
    memcpy(data_copy, data, length);
    
    NetworkMessage msg = {data_copy, length};
    
    // 尝试将消息放入队列，不阻塞
    if (xQueueSend(network_send_queue_, &msg, 0) != pdTRUE) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "网络发送队列已满，丢弃消息");
        free(data_copy);
    } else {
        ESP_LOGD(ROBOT_CONTROLLER_TAG, "消息已加入发送队列，长度: %zu bytes，来源: PSRAM", length);
    }
}

void ElegooRobotController::TcpServerTaskWrapper(void* parameter) {
    static_cast<ElegooRobotController*>(parameter)->TcpServerTask();
}

void ElegooRobotController::TcpServerTask() {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    
    // 固定大小缓冲区
    const size_t buffer_size = 2048;  // 2KB固定缓冲区
    uint8_t* buffer = (uint8_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buffer == nullptr) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "TCP任务中分配缓冲区内存失败，任务退出");
        vTaskDelete(NULL);
        return;
    }
    size_t buffer_pos = 0;  // 缓冲区当前位置

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "TCP服务器任务开始运行");

    while (network_running_) {
        tcp_client_fd_ = accept(tcp_socket_fd_, (struct sockaddr*)&client_addr, &client_addr_len);
        if (tcp_client_fd_ >= 0) {
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "客户端连接: %s:%d", client_ip, ntohs(client_addr.sin_port));

            // 设置socket接收超时
            struct timeval timeout;
            timeout.tv_sec = 5;  // 5秒超时
            timeout.tv_usec = 0;
            if (setsockopt(tcp_client_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
                ESP_LOGW(ROBOT_CONTROLLER_TAG, "设置socket超时失败: %s", strerror(errno));
            }

            StopUdpBroadcastTask();
            buffer_pos = 0;  // 重置缓冲区位置

            // 处理客户端消息
            while (network_running_) {
                // 检查缓冲区是否已满，如果满了则重置
                if (buffer_pos >= buffer_size * 0.9) {
                    ESP_LOGW(ROBOT_CONTROLLER_TAG, "缓冲区使用率过高，重置缓冲区");
                    buffer_pos = 0;
                }
                
                int len = recv(tcp_client_fd_, buffer + buffer_pos, buffer_size - buffer_pos, 0);
                if (len <= 0) {
                    if (len == 0) {
                        ESP_LOGI(ROBOT_CONTROLLER_TAG, "客户端断开连接");
                        break;
                    } else {
                        // 检查是否是超时或非阻塞错误
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            vTaskDelay(10 / portTICK_PERIOD_MS);  // 短暂延迟后继续
                            continue;
                        } else {
                            ESP_LOGE(ROBOT_CONTROLLER_TAG, "接收数据错误: %s", strerror(errno));
                            break;
                        }
                    }
                }
                
                buffer_pos += len;
                // 移除详细的数据接收日志
                
                // 尝试解析协议包
                size_t processed = 0;
                while (processed < buffer_pos) {
                    size_t bytes_consumed = 0;
                    bool parse_result = ParseProtocolPacket(buffer + processed, buffer_pos - processed, bytes_consumed);
                    
                    if (parse_result && bytes_consumed > 0) {
                        // 成功解析了一个包
                        processed += bytes_consumed;
                        // 协议包解析成功
                    } else if (parse_result && bytes_consumed == 0) {
                        // 这种情况不应该发生，防止无限循环
                        ESP_LOGW(ROBOT_CONTROLLER_TAG, "解析成功但未消耗字节，跳过1字节");
                        processed += 1;
                    } else {
                        // 解析失败，可能是数据不完整或无效数据
                        if (bytes_consumed > 0) {
                            // 跳过无效数据
                            processed += bytes_consumed;
                            // 跳过无效数据
                        } else {
                            // 数据不完整，等待更多数据
                            // 数据不完整，等待更多数据
                            break;
                        }
                    }
                }
                
                // 移动未处理的数据到缓冲区开头
                if (processed > 0 && processed < buffer_pos) {
                    size_t remaining = buffer_pos - processed;
                    memmove(buffer, buffer + processed, remaining);
                    buffer_pos = remaining;
                    ESP_LOGD(ROBOT_CONTROLLER_TAG, "移动 %zu 字节未处理数据到缓冲区开头", remaining);
                } else if (processed >= buffer_pos) {
                    // 所有数据都已处理
                    buffer_pos = 0;
                }
            }
            
            // 客户端断开连接时停止视频流
            StopVideoStream();
            
            close(tcp_client_fd_);
            tcp_client_fd_ = -1;
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "客户端连接已关闭");

            if (network_running_) {
                StartUdpBroadcastTask();
            }
        }
        
        vTaskDelay(UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
    }

    free(buffer);  // 释放堆内存
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "TCP服务器任务结束");
    vTaskDelete(NULL);
}

void ElegooRobotController::UdpBroadcastTaskWrapper(void* parameter) {
    static_cast<ElegooRobotController*>(parameter)->UdpBroadcastTask();
}

void ElegooRobotController::UdpBroadcastTask() {
    int broadcast_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (broadcast_socket < 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UDP广播socket创建失败");
        vTaskDelete(NULL);
        return;
    }

    int broadcast = 1;
    if (setsockopt(broadcast_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UDP广播socket选项设置失败");
        close(broadcast_socket);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in broadcast_addr;
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(udp_port_);
    broadcast_addr.sin_addr.s_addr = INADDR_BROADCAST;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UDP广播任务开始运行，端口: %d", udp_port_);

    while (udp_running_ && network_running_) {
        // 获取当前IP地址
        char current_ip[16] = "0.0.0.0";
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            esp_ip4addr_ntoa(&ip_info.ip, current_ip, sizeof(current_ip));
        }
        
        char msg[128];
        int msg_len = snprintf(msg, sizeof(msg), 
            "{\"ip\":\"%s\",\"tcp_port\":%d,\"ap_ssid\":\"Elegoo-RobotCar5\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}", 
            current_ip, tcp_port_, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        if (msg_len > 0 && msg_len < sizeof(msg)) {
            sendto(broadcast_socket, msg, msg_len, 0, 
                   (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr));
        }
        
        vTaskDelay(pdMS_TO_TICKS(UDP_BROADCAST_INTERVAL_MS));
    }

    close(broadcast_socket);
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UDP广播任务结束");
    vTaskDelete(NULL);
}

void ElegooRobotController::StartUdpBroadcastTask() {
    if (udp_running_ && udp_task_handle_) return;
    
    StopUdpBroadcastTask();
    if (!network_running_) return;
    
    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < 6144) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "内存不足，跳过UDP广播任务启动。可用内存: %u bytes", 
                 (unsigned int)free_heap);
        return;
    }
    
    udp_running_ = true;
    if (xTaskCreate(UdpBroadcastTaskWrapper, "udp_broadcast_task", UDP_TASK_STACK_SIZE, 
                   this, UDP_TASK_PRIORITY, &udp_task_handle_) != pdPASS) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UDP广播任务创建失败");
        udp_running_ = false;
        udp_task_handle_ = nullptr;
    }
}

void ElegooRobotController::StopUdpBroadcastTask() {
    if (udp_running_) {
        udp_running_ = false;
        if (udp_task_handle_) {
            vTaskDelay(NETWORK_TASK_STOP_DELAY_MS / portTICK_PERIOD_MS);
            udp_task_handle_ = nullptr;
        }
    }
}

bool ElegooRobotController::ConfigureUart(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,  // 设置RX流控制阈值，默认值为122
        .source_clk = UART_SCLK_DEFAULT,  // 使用默认时钟源
    };

    esp_err_t ret = uart_driver_install(uart_port_, buffer_size_ * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UART驱动安装失败: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_param_config(uart_port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UART参数配置失败: %s", esp_err_to_name(ret));
        uart_driver_delete(uart_port_);
        return false;
    }

    ret = uart_set_pin(uart_port_, tx_pin, rx_pin, rts_pin, cts_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UART引脚配置失败: %s", esp_err_to_name(ret));
        uart_driver_delete(uart_port_);
        return false;
    }

    return true;
}

bool ElegooRobotController::StartUartTask() {
    uart_task_running_ = true;
    if (xTaskCreate(UartReceiveTaskWrapper, "robot_uart_task", NETWORK_TASK_STACK_SIZE, 
                   this, NETWORK_TASK_PRIORITY, &uart_task_handle_) != pdPASS) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UART接收任务创建失败");
        uart_task_running_ = false;
        return false;
    }
    return true;
}


void ElegooRobotController::SendRawBytes(const uint8_t* data, size_t length) {
    if (!initialized_) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "控制器未初始化");
        return;
    }

    if (!data || length == 0) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "无效的字节数据或长度为0");
        return;
    }

    int bytes_written = uart_write_bytes(uart_port_, data, length);
    if (bytes_written == length) {
        ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送串口数据成功，长度: %zu bytes", length);
    } else {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "发送串口数据失败，期望: %zu bytes，实际: %d bytes", length, bytes_written);
    }
}


/// @brief 执行待机命令
void ElegooRobotController::ExecuteStandby() {
    
    uint8_t command_id[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    // 构建模式命令数据包: Command ID (6字节) + Type (1字节) + Mode (1字节)
    uint8_t command_data[8];
    memcpy(command_data, command_id, 6);
    command_data[6] = EXECUTE_MODE;
    command_data[7] = 0x00; // 待机模式
    
    SendProtocolPacket(PROTOCOL_TYPE_COMMAND, command_data, sizeof(command_data));
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送待机命令");
}

/// @brief 移动命令
/// @param dir_index 移动方向索引,0:停止,1:前进,2:后退,3:左转,4:右转,5:左前,6:右前,7:左后,8:右后
/// @param speed 移动速度 0-255
void ElegooRobotController::ExecuteMoveCommand(int dir_index, int speed) {
    
    uint8_t command_id[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    // 构建移动命令数据包: Command ID (6字节) + Type (1字节) + Dir (1字节) + Speed (1字节)
    uint8_t command_data[9];
    memcpy(command_data, command_id, 6);
    command_data[6] = EXECUTE_MOVE;
    command_data[7] = static_cast<uint8_t>(dir_index);
    command_data[8] = static_cast<uint8_t>(speed);
    
    SendProtocolPacket(PROTOCOL_TYPE_COMMAND, command_data, sizeof(command_data));
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送移动命令: 方向=%d, 速度=%d", dir_index, speed);
}

/// @brief 执行模式改变命令
/// @param mode_index 模式索引,0=待机,1=避障,2=循迹,3=跟随
void ElegooRobotController::ExecuteModeChangeCommand(int mode_index) {
    uint8_t command_id[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    // 构建模式命令数据包: Command ID (6字节) + Type (1字节) + Mode (1字节)
    uint8_t command_data[8];
    memcpy(command_data, command_id, 6);
    command_data[6] = EXECUTE_MODE;
    command_data[7] = static_cast<uint8_t>(mode_index);
    
    SendProtocolPacket(PROTOCOL_TYPE_COMMAND, command_data, sizeof(command_data));
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送模式改变命令: 模式=%d", mode_index);
}

/// @brief 执行电机控制命令
/// @param motor 电机索引,0=全部电机,1=左电机,2=右电机
/// @param speed 电机速度 0-255
/// @param direction 电机方向 0=不处理,1=正转,2=反转
void ElegooRobotController::ExecuteMotorControl(int motor, int speed, int direction) {
    uint8_t command_id[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    // 构建电机控制命令数据包: Command ID (6字节) + Type (1字节) + Motor (1字节) + Dir (1字节) + Speed (1字节)
    uint8_t command_data[10];
    memcpy(command_data, command_id, 6);
    command_data[6] = EXECUTE_MOTOR_CONTROL;
    command_data[7] = static_cast<uint8_t>(motor);
    command_data[8] = static_cast<uint8_t>(direction);
    command_data[9] = static_cast<uint8_t>(speed);
    
    SendProtocolPacket(PROTOCOL_TYPE_COMMAND, command_data, sizeof(command_data));
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送电机控制命令: 电机=%d, 方向=%d, 速度=%d", motor, direction, speed);
}

/// @brief 设置移动速度
/// @param speed 移动速度 0-255
void ElegooRobotController::ExecuteSetMoveSpeed(int speed) {
    // 通过电机控制命令设置所有电机速度
    ExecuteMotorControl(0, speed, 0); // 0=全部电机, 0=不处理方向
}

/// @brief 执行舵机控制命令
/// @param degree 舵机角度 0-180
void ElegooRobotController::ExecuteServoControl(int degree) {
    uint8_t command_id[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    // 构建舵机控制命令数据包: Command ID (6字节) + Type (1字节) + Servo (1字节) + Angle (2字节)
    uint8_t command_data[10];
    memcpy(command_data, command_id, 6);
    command_data[6] = EXECUTE_CHANGE_SERVO;
    command_data[7] = 1;    // 舵机编号1
    uint16_t angle = static_cast<uint16_t>(degree);
    memcpy(&command_data[8], &angle, 2); // 小端序存储角度
    
    SendProtocolPacket(PROTOCOL_TYPE_COMMAND, command_data, sizeof(command_data));
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送舵机控制命令: 角度=%d", degree);
}

/// @brief 执行传感器数据读取命令
/// @param sensor_type 传感器类型,1=拾取检测,2=超声波,3=红外左,4=红外中,5=红外右
void ElegooRobotController::ExecuteSensorRead(int sensor_type) {
    uint8_t command_id[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    // 构建传感器读取命令数据包: Command ID (6字节) + Type (1字节) + Sensor (1字节)
    uint8_t command_data[8];
    memcpy(command_data, command_id, 6);
    command_data[6] = EXECUTE_SENSING_DATA;
    command_data[7] = static_cast<uint8_t>(sensor_type);
    
    SendProtocolPacket(PROTOCOL_TYPE_COMMAND, command_data, sizeof(command_data));
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送传感器读取命令: 传感器类型=%d", sensor_type);
}

/// @brief WiFi连接成功后启动网络服务器
void ElegooRobotController::StartNetworkServerWhenReady() {
    if (!initialized_ || network_running_) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "控制器未初始化或网络服务器已运行");
        return;
    }

    // 获取设备IP地址
    char ip_str[16] = "0.0.0.0";
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
    }

    if (StartNetworkServer(ip_str, NETWORK_TCP_PORT_DEFAULT, NETWORK_UDP_PORT_DEFAULT)) {
        ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络服务器在IP地址 %s 上启动成功", ip_str);
    } else {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "网络服务器启动失败");
    }
}

// 视频流相关方法实现
void ElegooRobotController::SetCameraManager(CameraManager* camera_manager) {
    camera_manager_ = camera_manager;
    if (camera_manager_) {
        ESP_LOGI(ROBOT_CONTROLLER_TAG, "摄像头管理器设置成功");
    } else {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "摄像头管理器设置为空");
    }
}

bool ElegooRobotController::StartVideoStream() {
    if (video_streaming_) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "视频流已经在运行");
        return true;
    }

    if (!camera_manager_) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "摄像头管理器未设置，无法启动视频流");
        return false;
    }

    if (!camera_manager_->IsInitialized()) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "摄像头未初始化，无法启动视频流");
        return false;
    }

    if (tcp_client_fd_ < 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "没有TCP客户端连接，无法启动视频流");
        return false;
    }

    video_streaming_ = true;
    
    // 创建视频流任务
    if (xTaskCreate(VideoStreamTaskWrapper, "video_stream_task", 4096, 
                   this, 4, &video_stream_task_handle_) != pdPASS) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "视频流任务创建失败");
        video_streaming_ = false;
        return false;
    }

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "视频流启动成功");
    return true;
}

bool ElegooRobotController::StopVideoStream() {
    if (!video_streaming_) {
        return true;
    }

    video_streaming_ = false;
    
    if (video_stream_task_handle_) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        video_stream_task_handle_ = nullptr;
    }

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "视频流已停止");
    return true;
}

void ElegooRobotController::VideoStreamTaskWrapper(void* parameter) {
    static_cast<ElegooRobotController*>(parameter)->VideoStreamTask();
}

// 二进制协议
bool ElegooRobotController::SendProtocolPacket(ProtocolType type, const uint8_t* data, uint32_t data_length) {
    // 计算总包大小
    uint32_t total_size = PROTOCOL_HEADER_SIZE + PROTOCOL_LENGTH_SIZE + PROTOCOL_TYPE_SIZE + data_length;
    
    // 分配发送缓冲区
    uint8_t* packet_buffer = (uint8_t*)heap_caps_malloc(total_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!packet_buffer) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "协议包缓冲区分配失败");
        return false;
    }

    // 构建协议包
    size_t offset = 0;
    
    // 包头 0xFF 0x55
    packet_buffer[offset++] = PROTOCOL_HEADER_1;
    packet_buffer[offset++] = PROTOCOL_HEADER_2;
    
    // 数据长度（包括类型字段）
    uint32_t payload_length = PROTOCOL_TYPE_SIZE + data_length;
    memcpy(packet_buffer + offset, &payload_length, PROTOCOL_LENGTH_SIZE);
    offset += PROTOCOL_LENGTH_SIZE;
    
    // 数据类型
    packet_buffer[offset++] = static_cast<uint8_t>(type);
    
    // 数据内容
    if (data && data_length > 0) {
        memcpy(packet_buffer + offset, data, data_length);
    }

    // 使用队列发送数据包
    SendNetworkMessage(packet_buffer, total_size);
    
    free(packet_buffer);
    
    ESP_LOGD(ROBOT_CONTROLLER_TAG, "协议包已加入发送队列，类型: %d, 数据长度: %lu", type, (unsigned long)data_length);
    
    return true;
}

bool ElegooRobotController::SendVideoFrame(const uint8_t* frame_data, uint32_t frame_size) {
    if (!frame_data || frame_size == 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "视频帧数据无效");
        return false;
    }

    // 构建视频帧数据包
    // 数据格式: [frame_id(4字节)] + [timestamp(4字节)] + [width(2字节)] + [height(2字节)] + [format(1字节)] + [jpeg_data]
    uint32_t frame_id = esp_timer_get_time() / 1000; // 使用时间戳作为帧ID
    uint32_t timestamp = frame_id;
    uint16_t width = 320;  // QVGA宽度
    uint16_t height = 240; // QVGA高度
    uint8_t format = 1;    // 1 = JPEG格式

    uint32_t header_size = 4 + 4 + 2 + 2 + 1; // frame_id + timestamp + width + height + format
    uint32_t total_data_size = header_size + frame_size;
    
    uint8_t* video_data = (uint8_t*)heap_caps_malloc(total_data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!video_data) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "视频数据缓冲区分配失败");
        return false;
    }
    // 构建视频数据包
    size_t offset = 0;
    memcpy(video_data + offset, &frame_id, 4);
    offset += 4;
    memcpy(video_data + offset, &timestamp, 4);
    offset += 4;
    memcpy(video_data + offset, &width, 2);
    offset += 2;
    memcpy(video_data + offset, &height, 2);
    offset += 2;
    memcpy(video_data + offset, &format, 1);
    offset += 1;
    memcpy(video_data + offset, frame_data, frame_size);

    // 发送协议包
    bool result = SendProtocolPacket(PROTOCOL_TYPE_VIDEO_COMMAND, video_data, total_data_size);
    
    free(video_data);
    return result;
}

void ElegooRobotController::VideoStreamTask() {
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "视频流任务开始运行 - 二进制协议模式");
    
    uint32_t frame_counter = 0;
    
    while (video_streaming_ && tcp_client_fd_ >= 0) {
        // 获取摄像头帧
        camera_fb_t* fb = camera_manager_->GetFrame();
        if (fb == nullptr) {
            ESP_LOGE(ROBOT_CONTROLLER_TAG, "获取摄像头帧失败");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // 使用新的二进制协议发送视频帧
        bool success = SendVideoFrame(fb->buf, fb->len);
        
        // 释放摄像头帧
        camera_manager_->ReturnFrame(fb);

        if (!success) {
            ESP_LOGE(ROBOT_CONTROLLER_TAG, "发送视频帧失败");
            break;
        }

        frame_counter++;
        //ESP_LOGD(ROBOT_CONTROLLER_TAG, "发送视频帧 %lu，大小: %u 字节", (unsigned long)frame_counter, (unsigned int)fb->len);

        // 控制帧率
        vTaskDelay(33 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "视频流任务结束");
    vTaskDelete(NULL);
}

// 协议解析实现
bool ElegooRobotController::ParseProtocolPacket(const uint8_t* buffer, size_t buffer_size, size_t& bytes_consumed) {
    bytes_consumed = 0;
    
    // 显示完整的消息内容用于调试
    ESP_LOGD(ROBOT_CONTROLLER_TAG, "解析协议包，缓冲区大小: %zu 字节", buffer_size);
    
    // 检查是否有足够的数据来解析包头
    if (buffer_size < PROTOCOL_MIN_PACKET_SIZE) {
        return false;
    }
    
    // 检查协议头
    if (buffer[0] != PROTOCOL_HEADER_1 || buffer[1] != PROTOCOL_HEADER_2) {
        ESP_LOGD(ROBOT_CONTROLLER_TAG, "无效的协议头: 0x%02X 0x%02X (期望: 0x%02X 0x%02X)", 
                buffer[0], buffer[1], PROTOCOL_HEADER_1, PROTOCOL_HEADER_2);
        bytes_consumed = 1;  // 跳过一个字节继续寻找
        return true;
    }
    
    // 解析数据长度（小端序）- 这个长度包括类型字段
    uint32_t payload_length = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16) | (buffer[5] << 24);
    
    // 检查payload长度是否合理（防止恶意数据）
    if (payload_length > 1024 * 1024) {  // 最大1MB
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "payload长度过大: %u", (unsigned int)payload_length);
        bytes_consumed = 2;  // 跳过协议头
        return true;
    }
    
    // payload_length包括类型字段(1字节)，所以实际数据长度需要减1
    if (payload_length < PROTOCOL_TYPE_SIZE) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "payload长度太小: %u", (unsigned int)payload_length);
        bytes_consumed = 2;  // 跳过协议头
        return true;
    }
    
    uint32_t data_length = payload_length - PROTOCOL_TYPE_SIZE;
    
    // 计算完整包的大小
    size_t total_packet_size = PROTOCOL_HEADER_SIZE + PROTOCOL_LENGTH_SIZE + payload_length;
    
    // 检查是否有完整的包
    if (buffer_size < total_packet_size) {
        return false;  // 数据不完整，等待更多数据
    }
    
    // 解析协议类型
    ProtocolType type = static_cast<ProtocolType>(buffer[6]);
    
    // 获取数据指针
    const uint8_t* data = (data_length > 0) ? &buffer[7] : nullptr;
    
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "解析到协议包: 类型=0x%02X, 数据长度=%u", type, (unsigned int)data_length);
    
    // 处理协议包，传递原始完整包数据
    ProcessProtocolPacket(type, data, data_length, buffer, total_packet_size);
    
    bytes_consumed = total_packet_size;
    return true;
}

void ElegooRobotController::ProcessProtocolPacket(ProtocolType type, const uint8_t* data, uint32_t data_length, const uint8_t* original_packet, size_t packet_size) {
    switch (type) {
        case PROTOCOL_TYPE_COMMAND:
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "处理命令数据，长度: %u", (unsigned int)data_length);
            if (original_packet && packet_size > 0) {
                // 直接转发原始完整协议包到串口
                SendRawBytes(original_packet, packet_size);
            }
            break;
            
        case PROTOCOL_TYPE_VOICE:
            {
                ESP_LOGI(ROBOT_CONTROLLER_TAG, "收到语音数据，长度: %u，发送到小智服务器解析", (unsigned int)data_length);
                
                SendVoiceDataToServer(data, data_length);
                break;
            }
            
        case PROTOCOL_TYPE_HEARTBEAT:
            ESP_LOGD(ROBOT_CONTROLLER_TAG, "收到心跳包");
            break;
            
        case PROTOCOL_TYPE_VIDEO_COMMAND:
            ESP_LOGD(ROBOT_CONTROLLER_TAG, "收到视频帧数据，长度: %u", (unsigned int)data_length);
            // 这里收到的是控制命令，控制开始或结束发送视频帧
            if (data_length > 0 && data != nullptr) {
                if (data[0] == 0x01) {
                    StartVideoStream();
                } else {
                    StopVideoStream();
                }
            } else {
                ESP_LOGW(ROBOT_CONTROLLER_TAG, "视频命令数据为空或无效");
            }
            break;
            
        case PROTOCOL_TYPE_STATUS:
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "收到状态数据，长度: %u", (unsigned int)data_length);
            // 状态数据处理
            break;
            
        default:
            ESP_LOGW(ROBOT_CONTROLLER_TAG, "未知的协议类型: 0x%02X", type);
            break;
    }
}

void ElegooRobotController::SendVoiceDataToServer(const uint8_t* voice_data, uint32_t data_length) {
    if (!voice_data || data_length == 0) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "语音数据为空，无法发送");
        return;
    }

    // 检查数据长度是否为偶数（16位数据对齐）
    if (data_length % 2 != 0) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "语音数据长度不是16位对齐: %lu", (unsigned long)data_length);
        return;
    }
    //dang si nian fei guo ye kong
    ESP_LOGD(ROBOT_CONTROLLER_TAG, "接收到16位单声道PCM语音数据，长度: %lu，使用主框架发送到服务器", (unsigned long)data_length);
    
    // 检查可用内存
    size_t free_heap = esp_get_free_heap_size();
    const size_t MIN_FREE_HEAP = 2048; 
    if (free_heap < MIN_FREE_HEAP) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "内存不足，可用: %zu 字节，跳过此次语音数据", free_heap);
        return;
    }
    
    // 将uint8_t数据转换为int16_t PCM数据
    size_t total_samples = data_length / 2;
    const int16_t* pcm_input = reinterpret_cast<const int16_t*>(voice_data);
    
    // 创建PCM数据向量
    std::vector<int16_t> pcm_data(pcm_input, pcm_input + total_samples);
    
    auto& app = Application::GetInstance();
    auto& audio_service = app.GetAudioService();
    
    // 注释掉原来推送到编码队列的代码
    // 这个方法会将PCM数据推送到编码队列，然后由OpusCodecTask进行编码
    // audio_service.FeedExternalAudioData(std::move(pcm_data));
    
    // 改为播放接收到的音频：直接推送PCM数据到播放队列
    if (audio_service.PushPcmToPlaybackQueue(std::move(pcm_data), false)) {
        ESP_LOGD(ROBOT_CONTROLLER_TAG, "已将%zu字节PCM音频数据推送到播放队列", data_length);
    } else {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "推送PCM音频数据到播放队列失败");
    }
}

// 网络发送任务包装器
void ElegooRobotController::NetworkSendTaskWrapper(void* parameter) {
    static_cast<ElegooRobotController*>(parameter)->NetworkSendTask();
}

// 网络发送任务
void ElegooRobotController::NetworkSendTask() {
    //ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络发送任务开始运行");
    
    NetworkMessage msg;
    while (network_send_task_running_) {
        // 从队列中获取消息，等待100ms
        if (xQueueReceive(network_send_queue_, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 检查TCP客户端是否连接
            if (tcp_client_fd_ >= 0) {
                int bytes_sent = send(tcp_client_fd_, msg.data, msg.length, 0);
                if (bytes_sent == msg.length) {
                    ESP_LOGD(ROBOT_CONTROLLER_TAG, "发送socket字节数据成功，长度: %zu bytes", msg.length);
                } else if (bytes_sent >= 0) {
                    ESP_LOGW(ROBOT_CONTROLLER_TAG, "socket数据发送不完整，期望: %zu bytes，实际: %d bytes", msg.length, bytes_sent);
                } else {
                    ESP_LOGE(ROBOT_CONTROLLER_TAG, "socket数据发送失败，错误码: %d", errno);
                }
            } else {
                ESP_LOGD(ROBOT_CONTROLLER_TAG, "TCP客户端未连接，丢弃消息");
            }
            
            // 释放消息内存
            if (msg.data != nullptr) {
                free(msg.data);
            }
        }
    }
    
    //ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络发送任务结束");
    vTaskDelete(NULL);
}

// 启动网络发送任务
void ElegooRobotController::StartNetworkSendTask() {
    if (network_send_task_running_) {
        //ESP_LOGW(ROBOT_CONTROLLER_TAG, "网络发送任务已经运行");
        return;
    }
    
    network_send_task_running_ = true;
    if (xTaskCreate(NetworkSendTaskWrapper, "network_send_task", 2048, 
                   this, 5, &network_send_task_handle_) != pdPASS) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "网络发送任务创建失败");
        network_send_task_running_ = false;
        network_send_task_handle_ = nullptr;
    } else {
        ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络发送任务启动成功");
    }
}
// 停止网络发送任务
void ElegooRobotController::StopNetworkSendTask() {
    if (!network_send_task_running_) return;
    
    network_send_task_running_ = false;
    if (network_send_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(200));  // 等待任务结束
        network_send_task_handle_ = nullptr;
    }
    
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络发送任务已停止");
}