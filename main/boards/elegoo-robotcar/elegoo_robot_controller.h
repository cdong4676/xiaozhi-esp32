#pragma once

#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <string>
#include <functional>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <esp_mac.h>
#include <time.h>
#include <errno.h>
#include "config.h"

#define ROBOT_CONTROLLER_TAG "ElegooRobotController"


/**
 * Elegoo机器人小车通讯控制类
 * 负责UART通讯、JSON协议命令发送和消息处理
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
    void SendNetworkMessage(const char* message);
    bool IsNetworkServerRunning() const { return network_running_; }

    // 基础通讯功能
    void SendRawMessage(const char* message);
    void SendJsonCommand(const char* block_id, int n, int d1 = -1, int d2 = -1, int d3 = -1);

    // 状态查询
    bool IsInitialized() const { return initialized_; }

    // 执行控制命令
    void ExecuteStandby();
    void ExecuteMoveCommand(int dir_index, int speed);
    void ExecuteModeChangeCommand(int mode_index);
    void ExecuteSetMoveSpeed(int speed);
    void ExecuteMotorControl(int motor, int speed, int direction);
    void ExecuteServoControl(int degree);

private:
    // UART配置
    uart_port_t uart_port_;
    int buffer_size_;
    bool initialized_;

    // 网络配置
    int tcp_socket_fd_;
    int tcp_client_fd_;
    std::string server_ip_;
    uint16_t tcp_port_;
    uint16_t udp_port_;
    bool network_running_;
    bool udp_running_;  // UDP任务运行状态标志位

    // 任务控制
    TaskHandle_t uart_task_handle_;
    bool uart_task_running_;
    TaskHandle_t tcp_task_handle_;
    TaskHandle_t udp_task_handle_;


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

    // 消息处理
    void HandleStatusMessage(const std::string& status);
    void HandleSensorMessage(const std::string& sensor_data);
    void HandleErrorMessage(const std::string& error_info);
    void HandleAckMessage(const std::string& ack_command);

    // 工具方法
    std::string TrimMessage(const std::string& message);
};