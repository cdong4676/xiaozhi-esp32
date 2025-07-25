#include "elegoo_robot_controller.h"
#include <cstring>
#include <algorithm>

ElegooRobotController::ElegooRobotController()
    : uart_port_(UART_NUM_0), buffer_size_(NETWORK_BUFFER_SIZE), initialized_(false)
    , tcp_socket_fd_(-1), tcp_client_fd_(-1), tcp_port_(NETWORK_TCP_PORT_DEFAULT), udp_port_(NETWORK_UDP_PORT_DEFAULT)
    , network_running_(false), udp_running_(false)
    , uart_task_handle_(nullptr), uart_task_running_(false), tcp_task_handle_(nullptr), udp_task_handle_(nullptr)
{
}

ElegooRobotController::~ElegooRobotController() {
    Shutdown();
}

bool ElegooRobotController::Initialize(uart_port_t uart_port, int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin, int buffer_size) {
    if (initialized_) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "控制器已经初始化");
        return true;
    }

    uart_port_ = uart_port;
    buffer_size_ = buffer_size;

    // 配置并安装UART
    if (!ConfigureUart(baud_rate, tx_pin, rx_pin, rts_pin, cts_pin)) {
        return false;
    }

    // 启动UART接收任务
    if (!StartUartTask()) {
        uart_driver_delete(uart_port_);
        return false;
    }
    initialized_ = true;
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "机器人控制器初始化成功，UART端口: %d, 波特率: %d", uart_port_, baud_rate);
    return true;
}

void ElegooRobotController::Shutdown() {
    if (!initialized_) {
        return;
    }

    // 停止网络服务器
    StopNetworkServer();

    // 停止UART接收任务
    if (uart_task_running_) {
        uart_task_running_ = false;
        if (uart_task_handle_ != nullptr) {
            vTaskDelay(NETWORK_CLIENT_DISCONNECT_DELAY_MS / portTICK_PERIOD_MS);
            uart_task_handle_ = nullptr;
        }
    }

    // 删除UART驱动
    uart_driver_delete(uart_port_);
    
    initialized_ = false;
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "机器人控制器已关闭");
}

// UART任务相关实现
void ElegooRobotController::UartReceiveTaskWrapper(void* parameter) {
    ElegooRobotController* instance = static_cast<ElegooRobotController*>(parameter);
    instance->UartReceiveTask();
}

void ElegooRobotController::UartReceiveTask() {
    uint8_t* data = (uint8_t*)malloc(buffer_size_);
    if (data == nullptr) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UART接收缓冲区内存分配失败");
        return;
    }
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UART接收任务开始运行");

    while (uart_task_running_) {
        int len = uart_read_bytes(uart_port_, data, buffer_size_ - 1, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
        
        if (len > 0) {
            data[len] = '\0';
            std::string received_message((char*)data, len);
            
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "接收到消息: %s", received_message.c_str());
            SendNetworkMessage(received_message.c_str());
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(data);
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UART接收任务结束");
    vTaskDelete(NULL);
}

std::string ElegooRobotController::TrimMessage(const std::string& message) {
    std::string trimmed = message;
    trimmed.erase(trimmed.find_last_not_of(" \n\r\t") + 1);
    trimmed.erase(0, trimmed.find_first_not_of(" \n\r\t"));
    return trimmed;
}

// 网络通讯功能实现
bool ElegooRobotController::StartNetworkServer(const char* ip, uint16_t tcp_port, uint16_t udp_port) {
    if (network_running_) {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "网络服务器已经运行");
        return true;
    }

    server_ip_ = ip;
    tcp_port_ = tcp_port;
    udp_port_ = udp_port;

    // 创建并配置TCP服务器
    tcp_socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (tcp_socket_fd_ < 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "TCP socket创建失败");
        return false;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(tcp_port_);
    inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr.s_addr);

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
    BaseType_t tcp_ret = xTaskCreate(TcpServerTaskWrapper, "tcp_server_task", 
                                    NETWORK_TASK_STACK_SIZE, this, NETWORK_TASK_PRIORITY, &tcp_task_handle_);
    if (tcp_ret != pdPASS) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "TCP服务器任务创建失败");
        StopNetworkServer();
        return false;
    }

    // 启动UDP广播任务
    StartUdpBroadcastTask();

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络服务器启动成功，TCP端口: %d, UDP端口: %d", tcp_port_, udp_port_);
    return true;
}

void ElegooRobotController::StopNetworkServer() {
    if (!network_running_) {
        return;
    }

    network_running_ = false;

    // 停止UDP广播任务
    StopUdpBroadcastTask();

    // 关闭客户端连接
    if (tcp_client_fd_ >= 0) {
        close(tcp_client_fd_);
        tcp_client_fd_ = -1;
    }

    // 关闭TCP服务器socket
    if (tcp_socket_fd_ >= 0) {
        close(tcp_socket_fd_);
        tcp_socket_fd_ = -1;
    }

    // 等待TCP任务结束
    if (tcp_task_handle_) {
        vTaskDelay(NETWORK_CLIENT_RECONNECT_DELAY_MS / portTICK_PERIOD_MS);
        tcp_task_handle_ = nullptr;
    }

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "网络服务器已停止");
}

void ElegooRobotController::SendNetworkMessage(const char* message) {
    if (tcp_client_fd_ >= 0) {
        send(tcp_client_fd_, message, strlen(message), 0);
        ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送网络消息: %s", message);
    } else {
        ESP_LOGW(ROBOT_CONTROLLER_TAG, "没有客户端连接，无法发送消息");
    }
}

// TCP服务器任务
void ElegooRobotController::TcpServerTaskWrapper(void* parameter) {
    ElegooRobotController* instance = static_cast<ElegooRobotController*>(parameter);
    instance->TcpServerTask();
}

void ElegooRobotController::TcpServerTask() {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char buffer[128];

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "TCP服务器任务开始运行");

    while (network_running_) {
        tcp_client_fd_ = accept(tcp_socket_fd_, (struct sockaddr*)&client_addr, &client_addr_len);
        if (tcp_client_fd_ >= 0) {
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "客户端连接: %s:%d", client_ip, ntohs(client_addr.sin_port));

            StopUdpBroadcastTask(); // 客户端连接时停止UDP广播

            // 处理客户端消息
            while (network_running_) {
                int len = recv(tcp_client_fd_, buffer, sizeof(buffer) - 1, 0);
                if (len <= 0) {
                    if (len == 0) {
                        ESP_LOGI(ROBOT_CONTROLLER_TAG, "客户端断开连接");
                    } else {
                        ESP_LOGI(ROBOT_CONTROLLER_TAG, "接收数据错误");
                    }
                    break;
                }
                
                buffer[len] = '\0';
                ESP_LOGI(ROBOT_CONTROLLER_TAG, "接收网络消息: %s", buffer);
                SendRawMessage(buffer);
            }
            
            close(tcp_client_fd_);
            tcp_client_fd_ = -1;
            ESP_LOGI(ROBOT_CONTROLLER_TAG, "客户端连接已关闭");

            if (network_running_) {
                StartUdpBroadcastTask(); // 客户端断开时重新启动UDP广播
            }
        }
        
        vTaskDelay(UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
    }

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "TCP服务器任务结束");
    vTaskDelete(NULL);
}

// UDP广播任务
void ElegooRobotController::UdpBroadcastTaskWrapper(void* parameter) {
    ElegooRobotController* instance = static_cast<ElegooRobotController*>(parameter);
    instance->UdpBroadcastTask();
}

void ElegooRobotController::UdpBroadcastTask() {
    // 创建UDP广播socket
    int broadcast_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (broadcast_socket < 0) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UDP广播socket创建失败");
        vTaskDelete(NULL);
        return;
    }

    int broadcast = 1;
    setsockopt(broadcast_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in broadcast_addr;
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(udp_port_);
    broadcast_addr.sin_addr.s_addr = INADDR_BROADCAST;

    // 获取MAC地址用于广播消息
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UDP广播任务开始运行");

    while (udp_running_ && network_running_) {
        // 构建广播消息
        std::string broadcast_msg = "{\"ip\":\"" + server_ip_ + 
                                   "\",\"tcp_port\":" + std::to_string(tcp_port_) + 
                                   ",\"device_type\":\"Elegoo-RobotCar\"" +
                                   ",\"mac\":\"" + mac_str + "\"}";
        
        sendto(broadcast_socket, broadcast_msg.c_str(), broadcast_msg.length(), 0, 
               (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr));
        
        ESP_LOGD(ROBOT_CONTROLLER_TAG, "发送UDP广播: %s", broadcast_msg.c_str());
        vTaskDelay(pdMS_TO_TICKS(UDP_BROADCAST_INTERVAL_MS));
    }

    close(broadcast_socket);
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "UDP广播任务结束");
    vTaskDelete(NULL);
}

// UDP广播任务动态管理
void ElegooRobotController::StartUdpBroadcastTask() {
    if (udp_running_ && udp_task_handle_) return; // 已在运行
    
    StopUdpBroadcastTask(); // 先停止现有任务
    if (!network_running_) return;
    
    udp_running_ = true;
    if (xTaskCreate(UdpBroadcastTaskWrapper, "udp_broadcast_task", NETWORK_TASK_STACK_SIZE, 
                   this, NETWORK_TASK_PRIORITY, &udp_task_handle_) != pdPASS) {
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

// 私有辅助方法实现
bool ElegooRobotController::ConfigureUart(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
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
    BaseType_t task_ret = xTaskCreate(UartReceiveTaskWrapper, "robot_uart_task", 
                                     NETWORK_TASK_STACK_SIZE, this, NETWORK_TASK_PRIORITY, &uart_task_handle_);
    if (task_ret != pdPASS) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "UART接收任务创建失败");
        uart_task_running_ = false;
        return false;
    }
    return true;
}


///####### 下面是具体的命令 ########


/// @brief 发送字符串消息到串口
/// @param message 
void ElegooRobotController::SendRawMessage(const char* message) {
    if (!initialized_) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "控制器未初始化");
        return;
    }

    uint8_t len = strlen(message);
    uart_write_bytes(uart_port_, message, len);
    ESP_LOGI(ROBOT_CONTROLLER_TAG, "发送命令: %s", message);
}

void ElegooRobotController::SendJsonCommand(const char* block_id, int n, int d1, int d2, int d3) {
    if (!initialized_) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "控制器未初始化");
        return;
    }

    cJSON* json = cJSON_CreateObject();
    if (!json) {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "JSON对象创建失败");
        return;
    }

    // 添加JSON字段
    if (block_id) cJSON_AddStringToObject(json, "H", block_id);
    cJSON_AddNumberToObject(json, "N", n);
    if (d1 >= 0) cJSON_AddNumberToObject(json, "D1", d1);
    if (d2 >= 0) cJSON_AddNumberToObject(json, "D2", d2);
    if (d3 >= 0) cJSON_AddNumberToObject(json, "D3", d3);
    
    char* json_string = cJSON_PrintUnformatted(json);
    if (json_string) {
        SendRawMessage(json_string);
        free(json_string);
    } else {
        ESP_LOGE(ROBOT_CONTROLLER_TAG, "JSON字符串生成失败");
    }
    
    cJSON_Delete(json);
}

/// @brief 执行待机命令
void ElegooRobotController::ExecuteStandby() {
    SendJsonCommand(nullptr, 110, 0);
}

/// @brief 移动命令
/// @param dir_index 移动方向索引,0:前进,1:后退,2:左转,3:右转,4:左转前,5:左上,6:左下,7:右上,8:右下,9:停止
/// @param speed 移动速度 0-255
void ElegooRobotController::ExecuteMoveCommand(int dir_index, int speed) {
    SendJsonCommand(nullptr, 102, dir_index, speed);
}

/// @brief 执行模式改变命令
/// @param mode_index 模式索引,1 = 循迹模式  2=避障模式  3=跟随模式
void ElegooRobotController::ExecuteModeChangeCommand(int mode_index) {
    SendJsonCommand(nullptr, 101, mode_index);
}

/// @brief 执行电机控制命令
/// @param motor 电机索引,0=全部电机,1=左电机,2=右电机
/// @param speed 电机速度 0-255
/// @param direction 电机方向 1=顺时针,2=逆时针
void ElegooRobotController::ExecuteMotorControl(int motor, int speed, int direction) {
    SendJsonCommand(nullptr, 1, motor, speed, direction);
}

/// @brief 设置移动速度
/// @param speed 移动速度 0-255
void ElegooRobotController::ExecuteSetMoveSpeed(int speed) {
    SendJsonCommand(nullptr, 4, speed, speed);
}

/// @brief 执行舵机控制命令
/// @param servo_index 舵机索引,0=全部舵机,1=左舵机,2=右舵机
/// @param degree 舵机角度 0-180
void ElegooRobotController::ExecuteServoControl( int degree) {
    SendJsonCommand(nullptr, 5, 1, degree);
}