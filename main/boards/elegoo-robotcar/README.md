<p align="center">
  <img width="80%" align="center" src="../../../docs/V1/elegoo-robotcar.png" alt="logo">
</p>
  <h1 align="center">
  Elegoo Robot Car
</h1>

## 简介

Elegoo Robot Car 是一个基于 ESP32 的智能机器车项目，集成了小智AI语音助手。该项目支持摄像头视频流、底盘控制、语音交互等功能。

## 硬件要求

- ESP32-S3 开发板
- 摄像头模块 (OV2640)
- LCD显示屏 (ST7789, 240x240)
- Elegoo机器车底盘
- 音频模块 (麦克风和扬声器)

## 功能特性

### 1. 底盘控制
- 前进、后退、左转、右转
- 速度控制 (0-255)
- 电机独立控制
- 舵机云台控制 (0-180度)
- 多种工作模式：巡线、避障、跟随

### 2. 摄像头功能
- 实时视频流 (MJPEG格式)
- 图像捕获和AI分析
- 水平镜像和垂直翻转
- 多种分辨率支持
- Web端视频流访问

### 3. 网络通信
- WiFi连接
- TCP/UDP服务器
- HTTP视频流服务器
- 实时数据传输

### 4. 语音交互
- 语音唤醒
- 语音识别
- 语音合成
- 小智AI对话

## 代码结构优化

### 优化内容
本项目经过了全面的代码结构优化，提高了代码的可读性和维护性：

#### 1. 头文件重构
- 使用 `enum class` 替代传统枚举，提供更好的类型安全
- 添加详细的 Doxygen 风格注释
- 重新组织包含文件和类声明结构

#### 2. 函数模块化
- 将大型函数拆分为更小的功能单元
- `TcpServerTask` 被拆分为多个专门的辅助函数：
  - `AdjustBufferSize()` - 动态缓冲区管理
  - `ReceiveTcpData()` - TCP数据接收
  - `ProcessProtocolPackets()` - 协议包解析
  - `ConfigureClientSocket()` - 客户端socket配置
  - `HandleClientConnection()` - 客户端连接处理

#### 3. 注释改善
- 使用标准化的文档注释格式
- 详细说明函数参数和返回值
- 添加使用示例和注意事项

#### 4. 协议类型重构
```cpp
// 新的协议类型定义
enum class ProtocolType : uint8_t {
    COMMAND = 0x01,      ///< 控制命令
    VIDEO_COMMAND = 0x02, ///< 视频命令  
    VOICE = 0x03,        ///< 语音数据
    HEARTBEAT = 0x04,    ///< 心跳包
    STATUS = 0x05        ///< 状态信息
};

enum class BinaryCommandType : uint8_t {
    MODE_CONTROL = 0x01,   ///< 模式控制
    MOVE_CONTROL = 0x02,   ///< 移动控制
    MOTOR_CONTROL = 0x03,  ///< 电机控制
    SERVO_CONTROL = 0x04,  ///< 舵机控制
    SENSOR_DATA = 0x05     ///< 传感器数据
};
```

### 设计原则
- **单一职责原则**: 每个函数专注于单一功能
- **模块化设计**: 网络、控制、视频流功能分离
- **错误处理**: 完善的错误检查和日志记录
- **内存管理**: 动态缓冲区和资源自动管理

## MCP工具接口

### 底盘控制工具

| 工具名称 | 描述 | 参数 |
|---------|------|------|
| `self.chassis.move_by_direction` | 方向移动 | direction(1-9), speed(0-250) |
| `self.chassis.stop` | 停止 | 无 |
| `self.chassis.standby` | 待机 | 无 |
| `self.chassis.set_mode` | 设置模式 | mode(1=巡线,2=避障,3=跟随) |
| `self.chassis.control_motors` | 电机控制 | select(0-2), speed(0-255), direction(1-2) |
| `self.chassis.set_speed` | 设置速度 | speed(0-255) |
| `self.chassis.control_servo` | 舵机控制 | degree(0-180) |

### 摄像头控制工具

| 工具名称 | 描述 | 参数 |
|---------|------|------|
| `self.camera.capture_and_analyze` | 拍照识别 | question(字符串) |
| `self.camera.set_hmirror` | 水平镜像 | enable(布尔值) |
| `self.camera.set_vflip` | 垂直翻转 | enable(布尔值) |
| `self.camera.set_frame_size` | 设置分辨率 | size(字符串) |

## 网络服务

### HTTP视频流
- 访问地址: `http://[IP地址]/video_stream`
- 格式: MJPEG
- 实时传输

### TCP/UDP通信
- TCP端口: 100
- UDP端口: 8888
- 支持双向通信

## 配置说明

主要配置文件: `config.h`

### 引脚配置
```c
// UART通信
#define UART_ECHO_TXD GPIO_NUM_45
#define UART_ECHO_RXD GPIO_NUM_48

// 显示屏
#define DISPLAY_MOSI_PIN GPIO_NUM_47
#define DISPLAY_CLK_PIN GPIO_NUM_21
#define DISPLAY_DC_PIN GPIO_NUM_43
#define DISPLAY_CS_PIN GPIO_NUM_44

// 摄像头
#define CAMERA_XCLK GPIO_NUM_15
#define CAMERA_PCLK GPIO_NUM_13
// ... 其他摄像头引脚
```

### 网络配置
```c
#define NETWORK_TCP_PORT_DEFAULT 100
#define NETWORK_UDP_PORT_DEFAULT 8888
#define NETWORK_BUFFER_SIZE 512
```

## 使用方法

1. **硬件连接**: 按照引脚配置连接各模块
2. **WiFi配置**: 首次启动时配置WiFi连接
3. **语音交互**: 通过语音命令控制机器车
4. **Web访问**: 通过浏览器访问视频流
5. **MCP控制**: 通过小智AI的MCP工具控制

## 语音命令示例

- "向前走" / "后退" / "左转" / "右转"
- "停止" / "待机"
- "拍照" / "看看前面"
- "切换到避障模式"
- "云台转到90度"

## 开发说明

### 编译环境
- ESP-IDF v5.0+
- CMake构建系统

### 主要文件
- `elegoo_robotcar.cc`: 主控制类
- `elegoo_robot_controller.cc`: 底盘控制
- `camera_manager.cc`: 摄像头管理
- `elegoo_web_server.cc`: Web服务器
- `otto_emoji_display.cc`: 显示控制

### 内存优化
项目针对ESP32-S3进行了内存优化：
- 减少任务栈大小
- 优化缓冲区配置
- 使用PSRAM存储帧缓冲

## 故障排除

1. **摄像头初始化失败**: 检查引脚连接和电源
2. **WiFi连接问题**: 重置WiFi配置
3. **视频流无法访问**: 确认网络连接和防火墙设置
4. **底盘控制无响应**: 检查UART连接和波特率

## 版本信息

当前版本: 1.0.0

## 许可证

本项目遵循开源许可证，具体请参考项目根目录的LICENSE文件。

