#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define UART_ECHO_TXD GPIO_NUM_45
#define UART_ECHO_RXD GPIO_NUM_48
#define UART_ECHO_RTS (-1)
#define UART_ECHO_CTS (-1)

#define ECHO_UART_PORT_NUM      UART_NUM_1
#define ECHO_UART_BAUD_RATE     (9600)
#define BUF_SIZE                (1024)

#define AUDIO_INPUT_SAMPLE_RATE 16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
#define AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS GPIO_NUM_41
#define AUDIO_I2S_MIC_GPIO_SCK GPIO_NUM_2
#define AUDIO_I2S_MIC_GPIO_DIN GPIO_NUM_42
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_38
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_39
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_40

#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_46
#define DISPLAY_MOSI_PIN GPIO_NUM_47
#define DISPLAY_CLK_PIN GPIO_NUM_21
#define DISPLAY_DC_PIN GPIO_NUM_43
#define DISPLAY_RST_PIN GPIO_NUM_NC
#define DISPLAY_CS_PIN GPIO_NUM_44

#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR true
#define DISPLAY_RGB_ORDER LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 3

#define BOOT_BUTTON_GPIO GPIO_NUM_0

/* Camera PINs*/
#define CAMERA_XCLK      (GPIO_NUM_15)
#define CAMERA_PCLK      (GPIO_NUM_13)
#define CAMERA_VSYNC     (GPIO_NUM_6)
#define CAMERA_HSYNC     (GPIO_NUM_7)
#define CAMERA_D0        (GPIO_NUM_11)
#define CAMERA_D1        (GPIO_NUM_9)
#define CAMERA_D2        (GPIO_NUM_8)
#define CAMERA_D3        (GPIO_NUM_10)
#define CAMERA_D4        (GPIO_NUM_12)
#define CAMERA_D5        (GPIO_NUM_18)
#define CAMERA_D6        (GPIO_NUM_17)
#define CAMERA_D7        (GPIO_NUM_16)

#define CAMERA_PWDN      (GPIO_NUM_NC)
#define CAMERA_RESET     (GPIO_NUM_NC)

#define CAMERA_XCLK_FREQ (20000000)
#define LEDC_TIMER       (LEDC_TIMER_0)
#define LEDC_CHANNEL     (LEDC_CHANNEL_0)

#define CAMERA_SIOD      (GPIO_NUM_4)
#define CAMERA_SIOC      (GPIO_NUM_5)

#define CONFIG_ESP_FACE_DETECT_ENABLED 1
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 1

#define ELEGOO_ROBOT_VERSION "1.0.0"

#define NETWORK_TCP_PORT_DEFAULT        8080
#define NETWORK_UDP_PORT_DEFAULT        8888               
#define NETWORK_TASK_STACK_SIZE         3072                
#define UART_TASK_STACK_SIZE            3072                
#define NETWORK_TASK_PRIORITY           5                   
#define UDP_TASK_STACK_SIZE             2048               
#define UDP_TASK_PRIORITY               3                   
#define UDP_BROADCAST_INTERVAL_MS       2000               
#define UART_READ_TIMEOUT_MS            200                
#define NETWORK_CLIENT_DISCONNECT_DELAY_MS  100             
#define NETWORK_TASK_STOP_DELAY_MS      50                  

// Web 服务器配置
#define WEB_SERVER_RESPONSE_BUFFER_SIZE 64
#define WEB_SERVER_CONTENT_BUFFER_SIZE  1024                // 减少内容缓冲区
#define WEB_SERVER_DEFAULT_SPEED        1000
#define WEB_SERVER_DEFAULT_MOTOR_SPEED  100
#define WEB_SERVER_STACK_SIZE           4096                // 减少Web服务器栈大小 (4KB)
#define WEB_SERVER_INIT_RETRY_DELAY_MS  2000                // Web服务器初始化重试延迟


// 摄像头配置
#define CAMERA_PCLK_HZ                  (20 * 1000 * 1000)



// 网络任务配置
#define NETWORK_CLIENT_RECONNECT_DELAY_MS  150              // 客户端重连延迟

#endif  // _BOARD_CONFIG_H_
