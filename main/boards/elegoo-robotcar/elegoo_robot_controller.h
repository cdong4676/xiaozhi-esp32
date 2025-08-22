#ifndef ELEGOO_ROBOT_CONTROLLER_H
#define ELEGOO_ROBOT_CONTROLLER_H

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
#include "config.h"
#include "camera_manager.h"

#define ROBOT_CONTROLLER_TAG "ElegooRobotController"

// 网络发送队列消息结构
struct NetworkMessage {
    uint8_t* data;
    size_t length;
};

// 二进制通讯协议定义
#define PROTOCOL_HEADER_1 0xFF
#define PROTOCOL_HEADER_2 0x55
#define PROTOCOL_HEADER_SIZE 2
#define PROTOCOL_LENGTH_SIZE 4  // uint32_t
#define PROTOCOL_TYPE_SIZE 1    // uint8_t
#define PROTOCOL_MIN_PACKET_SIZE (PROTOCOL_HEADER_SIZE + PROTOCOL_LENGTH_SIZE + PROTOCOL_TYPE_SIZE)

// 协议类型定义
enum ProtocolType : uint8_t {
    PROTOCOL_TYPE_VIDEO_COMMAND = 0x01,
    PROTOCOL_TYPE_COMMAND = 0x02,
    PROTOCOL_TYPE_STATUS = 0x03,
    PROTOCOL_TYPE_HEARTBEAT = 0x04,
    PROTOCOL_TYPE_VOICE = 0x05
};

// 二进制命令类型定义
enum BinaryCommandType : uint8_t {
    EXECUTE_MODE = 0x01,           // 执行模式
    EXECUTE_MOVE = 0x02,           // 执行移动
    EXECUTE_MOTOR_CONTROL = 0x03,  // 执行电机控制
    EXECUTE_CHANGE_SERVO = 0x04,   // 执行舵机控制
    EXECUTE_SENSING_DATA = 0x05    // 执行获取传感器数据
};




/**
 * @brief Elegoo机器人小车通讯控制类 - 内存优化版本
 * 
 * 负责UART通讯、JSON协议命令发送和消息处理
 * 专为ESP32内存限制优化
 */
class ElegooRobotController {
public:
    ElegooRobotController();
    ~ElegooRobotController();

    // 初始化和清理
    bool Initialize(uart_port_t uart_port, int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin, int buffer_size);
    void Shutdown();

    // 网络通讯功能
    bool StartNetworkServer(const char* ip, uint16_t tcp_port = NETWORK_TCP_PORT_DEFAULT, uint16_t udp_port = NETWORK_UDP_PORT_DEFAULT);
    void StopNetworkServer();
    void SendNetworkMessage(const uint8_t* data, size_t length);
    bool IsNetworkServerRunning() const { return network_running_; }
    
    // WiFi连接成功后启动网络服务器
    void StartNetworkServerWhenReady();

    // 基础通讯功能
    void SendRawBytes(const uint8_t* data, size_t length);

    // 状态查询
    bool IsInitialized() const { return initialized_; }

    // 执行控制命令
    void ExecuteStandby();
    void ExecuteMoveCommand(int dir_index, int speed);
    void ExecuteModeChangeCommand(int mode_index);
    void ExecuteSetMoveSpeed(int speed);
    void ExecuteMotorControl(int motor, int speed, int direction);
    void ExecuteServoControl(int degree);
    void ExecuteSensorRead(int sensor_type);

    // 视频流控制
    void SetCameraManager(CameraManager* camera_manager);
    bool StartVideoStream();
    bool StopVideoStream();
    bool IsVideoStreamRunning() const { return video_streaming_; }

    // 二进制协议相关方法
    bool SendProtocolPacket(ProtocolType type, const uint8_t* data, uint32_t data_length);
    bool SendVideoFrame(const uint8_t* frame_data, uint32_t frame_size);
    
    // 协议解析方法
    bool ParseProtocolPacket(const uint8_t* buffer, size_t buffer_size, size_t& bytes_consumed);
    void ProcessProtocolPacket(ProtocolType type, const uint8_t* data, uint32_t data_length, const uint8_t* original_packet, size_t packet_size);
    
    // 语音数据处理
    void SendVoiceDataToServer(const uint8_t* voice_data, uint32_t data_length);
    

private:
    // UART配置
    uart_port_t uart_port_;
    int buffer_size_;
    bool initialized_;

    // 网络配置
    int tcp_socket_fd_;
    int tcp_client_fd_;
    uint16_t tcp_port_;
    uint16_t udp_port_;
    bool network_running_;
    bool udp_running_;  // UDP任务运行状态标志位

    // 任务控制
    TaskHandle_t uart_task_handle_;
    bool uart_task_running_;
    TaskHandle_t tcp_task_handle_;
    TaskHandle_t udp_task_handle_;

    // 视频流相关
    CameraManager* camera_manager_;
    bool video_streaming_;
    TaskHandle_t video_stream_task_handle_;
    
    // 网络发送队列
    QueueHandle_t network_send_queue_;
    TaskHandle_t network_send_task_handle_;
    bool network_send_task_running_;
    
    // 简化的缓冲区管理 - 使用固定大小


    // UART配置和任务管理
    bool ConfigureUart(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin);
    bool StartUartTask();

    // UART任务相关
    static void UartReceiveTaskWrapper(void* parameter);
    void UartReceiveTask();

    // 网络相关任务包装器
  static void TcpServerTaskWrapper(void* parameter);
  static void UdpBroadcastTaskWrapper(void* parameter);
  
  // 网络相关任务
  void TcpServerTask();
  void UdpBroadcastTask();
  
  // UDP广播任务动态管理
  void StartUdpBroadcastTask();
  void StopUdpBroadcastTask();

  // 视频流任务相关
  static void VideoStreamTaskWrapper(void* parameter);
  void VideoStreamTask();
  
  // 网络发送任务相关
  static void NetworkSendTaskWrapper(void* parameter);
  void NetworkSendTask();
  void StartNetworkSendTask();
  void StopNetworkSendTask();
  
  // 内存池管理方法


private:
    // 禁用拷贝构造和赋值
    ElegooRobotController(const ElegooRobotController&) = delete;
    ElegooRobotController& operator=(const ElegooRobotController&) = delete;

};

#endif // ELEGOO_ROBOT_CONTROLLER_H