#ifndef ELEGOO_ROBOT_CONTROLLER_H
#define ELEGOO_ROBOT_CONTROLLER_H

// 系统头文件
#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <errno.h>

// 项目头文件
#include "config.h"
#include "camera_manager.h"

// 日志标签
#define ROBOT_CONTROLLER_TAG "ElegooRobotController"

// ============================================================================
// 数据结构定义
// ============================================================================

/**
 * @brief 网络发送队列消息结构
 */
struct NetworkMessage {
    uint8_t* data;      ///< 消息数据指针
    size_t length;      ///< 消息长度
};

// ============================================================================
// 二进制通讯协议定义
// ============================================================================

// 协议头部定义
#define PROTOCOL_HEADER_1           0xFF
#define PROTOCOL_HEADER_2           0x55
#define PROTOCOL_HEADER_SIZE        2
#define PROTOCOL_LENGTH_SIZE        4    // uint32_t
#define PROTOCOL_TYPE_SIZE          1    // uint8_t
#define PROTOCOL_MIN_PACKET_SIZE    (PROTOCOL_HEADER_SIZE + PROTOCOL_LENGTH_SIZE + PROTOCOL_TYPE_SIZE)

/**
 * @brief 协议类型枚举
 */
enum class ProtocolType : uint8_t {
    VIDEO_COMMAND   = 0x01,     ///< 视频命令
    COMMAND         = 0x02,     ///< 普通命令
    STATUS          = 0x03,     ///< 状态信息
    HEARTBEAT       = 0x04,     ///< 心跳包
    VOICE           = 0x05      ///< 语音数据
};

/**
 * @brief 二进制命令类型枚举
 */
enum class BinaryCommandType : uint8_t {
    MODE_CONTROL    = 0x01,     ///< 执行模式控制
    MOVE_CONTROL    = 0x02,     ///< 执行移动控制
    MOTOR_CONTROL   = 0x03,     ///< 执行电机控制
    SERVO_CONTROL   = 0x04,     ///< 执行舵机控制
    SENSOR_DATA     = 0x05      ///< 执行获取传感器数据
};

// ============================================================================
// 主控制类定义
// ============================================================================

/**
 * @brief Elegoo机器人小车通讯控制类
 * 
 * 负责UART通讯、网络协议命令发送和消息处理
 * 专为ESP32内存限制优化，支持视频流和语音数据传输
 */
class ElegooRobotController {
public:
    // ========================================================================
    // 构造函数和析构函数
    // ========================================================================
    ElegooRobotController();
    ~ElegooRobotController();

    // ========================================================================
    // 初始化和生命周期管理
    // ========================================================================
    
    /**
     * @brief 初始化机器人控制器
     * @param uart_port UART端口号
     * @param baud_rate 波特率
     * @param tx_pin 发送引脚
     * @param rx_pin 接收引脚
     * @param rts_pin RTS引脚
     * @param cts_pin CTS引脚
     * @param buffer_size 缓冲区大小
     * @return 初始化是否成功
     */
    bool Initialize(uart_port_t uart_port, int baud_rate, int tx_pin, int rx_pin, 
                   int rts_pin, int cts_pin, int buffer_size);
    
    /**
     * @brief 关闭控制器并释放资源
     */
    void Shutdown();

    // ========================================================================
    // 网络通讯功能
    // ========================================================================
    
    /**
     * @brief 启动网络服务器
     * @param ip 服务器IP地址
     * @param tcp_port TCP端口号
     * @param udp_port UDP端口号
     * @return 启动是否成功
     */
    bool StartNetworkServer(const char* ip, 
                           uint16_t tcp_port = NETWORK_TCP_PORT_DEFAULT, 
                           uint16_t udp_port = NETWORK_UDP_PORT_DEFAULT);
    
    /**
     * @brief 停止网络服务器
     */
    void StopNetworkServer();
    
    /**
     * @brief 发送网络消息
     * @param data 消息数据
     * @param length 消息长度
     */
    void SendNetworkMessage(const uint8_t* data, size_t length);
    
    /**
     * @brief 检查网络服务器是否运行
     * @return 运行状态
     */
    bool IsNetworkServerRunning() const { return network_running_; }
    
    /**
     * @brief WiFi连接成功后启动网络服务器
     */
    void StartNetworkServerWhenReady();

    // ========================================================================
    // 基础通讯功能
    // ========================================================================
    
    /**
     * @brief 发送原始字节数据
     * @param data 数据指针
     * @param length 数据长度
     */
    void SendRawBytes(const uint8_t* data, size_t length);

    // ========================================================================
    // 状态查询
    // ========================================================================
    
    /**
     * @brief 检查控制器是否已初始化
     * @return 初始化状态
     */
    bool IsInitialized() const { return initialized_; }

    // ========================================================================
    // 机器人控制命令
    // ========================================================================
    
    void ExecuteStandby();                                          ///< 执行待机命令
    void ExecuteMoveCommand(int dir_index, int speed);              ///< 执行移动命令
    void ExecuteModeChangeCommand(int mode_index);                  ///< 执行模式切换命令
    void ExecuteSetMoveSpeed(int speed);                           ///< 设置移动速度
    void ExecuteMotorControl(int motor, int speed, int direction);  ///< 执行电机控制
    void ExecuteServoControl(int degree);                          ///< 执行舵机控制
    void ExecuteSensorRead(int sensor_type);                       ///< 读取传感器数据

    // ========================================================================
    // 视频流控制
    // ========================================================================
    
    /**
     * @brief 设置摄像头管理器
     * @param camera_manager 摄像头管理器指针
     */
    void SetCameraManager(CameraManager* camera_manager);
    
    /**
     * @brief 启动视频流
     * @return 启动是否成功
     */
    bool StartVideoStream();
    
    /**
     * @brief 停止视频流
     * @return 停止是否成功
     */
    bool StopVideoStream();
    
    /**
     * @brief 检查视频流是否运行
     * @return 运行状态
     */
    bool IsVideoStreamRunning() const { return video_streaming_; }

    // ========================================================================
    // 二进制协议处理
    // ========================================================================
    
    /**
     * @brief 发送协议数据包
     * @param type 协议类型
     * @param data 数据指针
     * @param data_length 数据长度
     * @return 发送是否成功
     */
    bool SendProtocolPacket(ProtocolType type, const uint8_t* data, uint32_t data_length);
    
    /**
     * @brief 发送视频帧
     * @param frame_data 帧数据
     * @param frame_size 帧大小
     * @return 发送是否成功
     */
    bool SendVideoFrame(const uint8_t* frame_data, uint32_t frame_size);
    
    /**
     * @brief 解析协议数据包
     * @param buffer 缓冲区
     * @param buffer_size 缓冲区大小
     * @param bytes_consumed 消耗的字节数
     * @return 解析是否成功
     */
    bool ParseProtocolPacket(const uint8_t* buffer, size_t buffer_size, size_t& bytes_consumed);
    
    /**
     * @brief 处理协议数据包
     * @param type 协议类型
     * @param data 数据指针
     * @param data_length 数据长度
     * @param original_packet 原始数据包
     * @param packet_size 数据包大小
     */
    void ProcessProtocolPacket(ProtocolType type, const uint8_t* data, uint32_t data_length, 
                              const uint8_t* original_packet, size_t packet_size);
    
    /**
     * @brief 发送语音数据到服务器
     * @param voice_data 语音数据
     * @param data_length 数据长度
     */
    void SendVoiceDataToServer(const uint8_t* voice_data, uint32_t data_length);
    

private:
    // ========================================================================
    // 成员变量
    // ========================================================================
    
    // UART配置
    uart_port_t uart_port_;         ///< UART端口号
    int buffer_size_;               ///< 缓冲区大小
    bool initialized_;              ///< 初始化状态

    // 网络配置
    int tcp_socket_fd_;             ///< TCP套接字文件描述符
    int tcp_client_fd_;             ///< TCP客户端文件描述符
    uint16_t tcp_port_;             ///< TCP端口号
    uint16_t udp_port_;             ///< UDP端口号
    bool network_running_;          ///< 网络服务运行状态
    bool udp_running_;              ///< UDP任务运行状态

    // 任务句柄
    TaskHandle_t uart_task_handle_;         ///< UART任务句柄
    bool uart_task_running_;                ///< UART任务运行状态
    TaskHandle_t tcp_task_handle_;          ///< TCP任务句柄
    TaskHandle_t udp_task_handle_;          ///< UDP任务句柄

    // 视频流相关
    CameraManager* camera_manager_;         ///< 摄像头管理器指针
    bool video_streaming_;                  ///< 视频流状态
    TaskHandle_t video_stream_task_handle_; ///< 视频流任务句柄
    
    // 网络发送队列
    QueueHandle_t network_send_queue_;      ///< 网络发送队列
    TaskHandle_t network_send_task_handle_; ///< 网络发送任务句柄
    bool network_send_task_running_;        ///< 网络发送任务运行状态
    
    // ========================================================================
    // 常量定义
    // ========================================================================
    
    // TCP缓冲区动态管理常量
    static constexpr size_t MIN_BUFFER_SIZE = 1024;        ///< 最小缓冲区大小 (1KB)
    static constexpr size_t MAX_BUFFER_SIZE = 4096;        ///< 最大缓冲区大小 (4KB)
    static constexpr double BUFFER_GROW_THRESHOLD = 0.8;   ///< 缓冲区扩展阈值 (80%)
    static constexpr double BUFFER_SHRINK_THRESHOLD = 0.3; ///< 缓冲区收缩阈值 (30%)

    // ========================================================================
    // 私有方法
    // ========================================================================
    
    // UART配置和管理
    bool ConfigureUart(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin);
    bool StartUartTask();

    // UART任务
    static void UartReceiveTaskWrapper(void* parameter);
    void UartReceiveTask();

    // 网络任务包装器
    static void TcpServerTaskWrapper(void* parameter);
    static void UdpBroadcastTaskWrapper(void* parameter);
    
    // 网络任务实现
    void TcpServerTask();
    void UdpBroadcastTask();
    
    // TCP服务器辅助方法
    void AdjustBufferSize(uint8_t*& buffer, size_t& current_size, size_t& peak_usage, size_t buffer_pos);
    int ReceiveTcpData(uint8_t* buffer, size_t& buffer_pos, size_t current_size, size_t& peak_usage);
    void ProcessProtocolPackets(uint8_t* buffer, size_t& buffer_pos);
    bool ConfigureClientSocket(int client_fd);
    void HandleClientConnection(int client_fd, uint8_t*& buffer, size_t& current_size, 
                               size_t& buffer_pos, size_t& peak_usage, uint32_t& resize_counter);
    
    // UDP广播任务管理
    void StartUdpBroadcastTask();
    void StopUdpBroadcastTask();

    // 视频流任务
    static void VideoStreamTaskWrapper(void* parameter);
    void VideoStreamTask();
    
    // 网络发送任务
    static void NetworkSendTaskWrapper(void* parameter);
    void NetworkSendTask();
    void StartNetworkSendTask();
    void StopNetworkSendTask();
    
    // ========================================================================
    // 禁用拷贝和赋值
    // ========================================================================
    ElegooRobotController(const ElegooRobotController&) = delete;
    ElegooRobotController& operator=(const ElegooRobotController&) = delete;

};

#endif // ELEGOO_ROBOT_CONTROLLER_H