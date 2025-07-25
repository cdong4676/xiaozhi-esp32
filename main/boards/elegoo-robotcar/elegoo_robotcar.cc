#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <wifi_station.h>
#include <driver/uart.h>
#include <cstring>
#include <string>
#include <esp_wifi.h>
#include <esp_event.h>

#include "application.h"
#include "audio_codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/lcd_display.h"
#include "otto_emoji_display.h"
#include "system_reset.h"
#include "wifi_board.h"
#include "camera_manager.h"
#include "elegoo_web_server.h"
#include "mcp_server.h"
#include "settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "elegoo_robot_controller.h"

#define TAG "ElegooRobotCar"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

class ElegooRobotCar : public WifiBoard {
private:
    LcdDisplay* display_;
    Button boot_button_;
    ElegooRobotController* robot_controller_;
    bool web_server_initialized_ = false;

    static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
    {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
            xTaskCreate(
                [](void* arg) {
                    ElegooRobotCar* instance = static_cast<ElegooRobotCar*>(arg);
                    
                    // 增加延迟时间，确保系统稳定
                    vTaskDelay(8000 / portTICK_PERIOD_MS);

                    if (!instance->web_server_initialized_) {
                        ESP_LOGI(TAG, "WiFi连接成功，初始化Elegoo机器人Web控制服务器");
                        
                        // 检查系统状态
                        ESP_LOGI(TAG, "启动前系统状态 - 可用内存: %u bytes", (unsigned int)esp_get_free_heap_size());
                        
                        esp_err_t ret = elegoo_web_server_init();
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Web服务器初始化失败: %s", esp_err_to_name(ret));
                        } else {
                            ESP_LOGI(TAG, "Web服务器初始化成功");
                            instance->web_server_initialized_ = true;
                        }
                    }

                    vTaskDelete(NULL);
                },
                "web_server_init",
                MAIN_TASK_STACK_SIZE, arg, MAIN_TASK_PRIORITY - 1, nullptr);  // 降低优先级
        }
    }
   
    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = CAMERA_PCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);
        
         display_ = new OttoEmojiDisplay(
            panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
            {
                .text_font = &font_puhui_16_4,
                .icon_font = &font_awesome_16_4,
                .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
            });
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting &&
                !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeWebServer() {
        ESP_LOGI(TAG, "注册WiFi事件处理器");
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
                                                 &wifi_event_handler, this));
    }

    /*
        Elegoo 机器人小车的底盘控制
        通过机器人控制器与底盘控制板通信
    */
    void InitializeRobotController() {
        robot_controller_ = new ElegooRobotController();
        
        bool success = robot_controller_->Initialize(
            ECHO_UART_PORT_NUM, 
            ECHO_UART_BAUD_RATE, 
            UART_ECHO_TXD, 
            UART_ECHO_RXD, 
            UART_ECHO_RTS, 
            UART_ECHO_CTS, 
            BUF_SIZE
        );
        
        if (!success) {
            ESP_LOGE(TAG, "机器人控制器初始化失败");
            delete robot_controller_;
            robot_controller_ = nullptr;
            return;
        }
        
        ESP_LOGI(TAG, "机器人控制器初始化完成");
    }

    void InitializeCamera() {
        camera_config_t camera_config = {};

        camera_config.pin_pwdn = CAMERA_PWDN;
        camera_config.pin_reset = CAMERA_RESET;
        camera_config.pin_xclk = CAMERA_XCLK;
        camera_config.pin_pclk = CAMERA_PCLK;
        camera_config.pin_sccb_sda = CAMERA_SIOD;
        camera_config.pin_sccb_scl = CAMERA_SIOC;

        camera_config.pin_d0 = CAMERA_D0;
        camera_config.pin_d1 = CAMERA_D1;
        camera_config.pin_d2 = CAMERA_D2;
        camera_config.pin_d3 = CAMERA_D3;
        camera_config.pin_d4 = CAMERA_D4;
        camera_config.pin_d5 = CAMERA_D5;
        camera_config.pin_d6 = CAMERA_D6;
        camera_config.pin_d7 = CAMERA_D7;

        camera_config.pin_vsync = CAMERA_VSYNC;
        camera_config.pin_href = CAMERA_HSYNC;
        camera_config.xclk_freq_hz = CAMERA_XCLK_FREQ;
        camera_config.ledc_timer = LEDC_TIMER;
        camera_config.ledc_channel = LEDC_CHANNEL;
        camera_config.fb_location = CAMERA_FB_IN_PSRAM;
        
        camera_config.sccb_i2c_port = I2C_NUM_0;
        
        camera_config.pixel_format = PIXFORMAT_RGB565;
        camera_config.frame_size = FRAMESIZE_240X240;
        camera_config.jpeg_quality = 12;
        camera_config.fb_count = 1;
        camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        
        // 使用单例模式初始化CameraManager
        auto& camera_manager = CameraManager::GetInstance();
        bool success = camera_manager.Initialize(camera_config);
        
        if (!success) {
            ESP_LOGE(TAG, "摄像头管理器初始化失败");
            return;
        }

        ESP_LOGI(TAG, "摄像头管理器初始化完成");
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        
        // 底盘控制工具 - 使用JSON协议
        mcp_server.AddTool("self.chassis.move_by_direction", 
            "根据方向和速度移动。direction:移动方向(1=前进, 2=后退, 3=左转,4=右转,5=左上,6=左下,7=右上,8=右下,9=停止);"
            "speed:速度(0-255,数值越大越快)", 
            PropertyList(std::vector<Property>{
            Property("direction", kPropertyTypeInteger, 1, 0,9),
            Property("speed", kPropertyTypeInteger, 200, 0,250)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int direction = properties["direction"].value<int>();
            int speed = properties["speed"].value<int>();
            robot_controller_->ExecuteMoveCommand(direction,speed);
            return true;
        });

        mcp_server.AddTool("self.chassis.stop", "停止", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            robot_controller_->ExecuteStandby();
            return true;
        });

        mcp_server.AddTool("self.chassis.standby", "待机", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            robot_controller_->ExecuteStandby();
            return true;
        });

        mcp_server.AddTool("self.chassis.set_mode", "设置功能模式。mode:(1=巡线,2=避障,3=跟随)", 
            PropertyList(std::vector<Property>{
            Property("mode", kPropertyTypeInteger, 1, 1,3)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int mode = properties["mode"].value<int>();
            robot_controller_->ExecuteModeChangeCommand(mode);
            return true;
        });

        mcp_server.AddTool("self.chassis.control_motors", 
            "控制电机。select:选择要控制的电机(0=同时控制左右电机,1=控制左电机,2=控制右电机);"
            "speed:速度(0-255,数值越大越快);"
            "direction:方向(1=顺时针,2=逆时针);", 
            PropertyList(std::vector<Property>{
            Property("select", kPropertyTypeInteger, 0, 0,2),
            Property("speed", kPropertyTypeInteger, 200, 0,255),
            Property("direction", kPropertyTypeInteger, 1, 1,2),
        }), [this](const PropertyList& properties) -> ReturnValue {
            int select = properties["select"].value<int>();
            int speed = properties["speed"].value<int>();
            int direction = properties["direction"].value<int>();
            
            robot_controller_->ExecuteMotorControl(select, speed, direction);
            return true;
        });

        mcp_server.AddTool("self.chassis.set_speed", "设置移动速度", PropertyList(std::vector<Property>{
            Property("speed", kPropertyTypeInteger, 200, 0,255)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int speed = properties["speed"].value<int>();
            robot_controller_->ExecuteSetMoveSpeed(speed);
            return true;
        });

        mcp_server.AddTool("self.chassis.control_servo", "控制云台舵机角度。degress:舵机角度(0-180)", PropertyList(std::vector<Property>{
            Property("degree", kPropertyTypeInteger, 90, 0,180)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int degree = properties["degree"].value<int>();
            // 限制角度范围
            degree = std::max(0, std::min(180, degree));
            robot_controller_->ExecuteServoControl(degree);
            return true;
        });

        // 摄像头控制工具
        

        mcp_server.AddTool("self.camera.set_hmirror", "设置水平镜像", PropertyList(std::vector<Property>{
            Property("enable", kPropertyTypeBoolean, false)
        }), [this](const PropertyList& properties) -> ReturnValue {
            auto& camera_manager = CameraManager::GetInstance();
            if (camera_manager.IsInitialized()) {
                bool enable = properties["enable"].value<bool>();
                camera_manager.SetHMirror(enable);
                return "设置成功";
            }
            return "摄像头未初始化";
        });

        mcp_server.AddTool("self.camera.set_vflip", "设置垂直翻转", PropertyList(std::vector<Property>{
            Property("enable", kPropertyTypeBoolean, false)
        }), [this](const PropertyList& properties) -> ReturnValue {
            auto& camera_manager = CameraManager::GetInstance();
            if (camera_manager.IsInitialized()) {
                bool enable = properties["enable"].value<bool>();
                camera_manager.SetVFlip(enable);
                return "设置成功";
            }
            return "摄像头未初始化";
        });

        mcp_server.AddTool("self.camera.get_frame", "获取摄像头帧", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            auto& camera_manager = CameraManager::GetInstance();
            if (camera_manager.IsInitialized()) {
                camera_fb_t* fb = camera_manager.GetFrame();
                if (fb) {
                    cJSON* json = cJSON_CreateObject();
                    cJSON_AddNumberToObject(json, "width", fb->width);
                    cJSON_AddNumberToObject(json, "height", fb->height);
                    cJSON_AddNumberToObject(json, "length", fb->len);
                    cJSON_AddNumberToObject(json, "format", fb->format);
                    
                    char* json_string = cJSON_Print(json);
                    std::string result(json_string);
                    free(json_string);
                    cJSON_Delete(json);
                    
                    camera_manager.ReturnFrame(fb);
                    return result;
                } else {
                    return "获取帧失败";
                }
            }
            return "摄像头未初始化";
        });

        mcp_server.AddTool("self.camera.set_frame_size", "设置帧大小", PropertyList(std::vector<Property>{
            Property("size", kPropertyTypeString, "240x240")
        }), [this](const PropertyList& properties) -> ReturnValue {
            auto& camera_manager = CameraManager::GetInstance();
            if (camera_manager.IsInitialized()) {
                std::string size_str = properties["size"].value<std::string>();
                framesize_t size = FRAMESIZE_240X240; // 默认值
                
                if (size_str == "96x96") size = FRAMESIZE_96X96;
                else if (size_str == "160x120") size = FRAMESIZE_QQVGA;
                else if (size_str == "176x144") size = FRAMESIZE_QCIF;
                else if (size_str == "240x176") size = FRAMESIZE_HQVGA;
                else if (size_str == "240x240") size = FRAMESIZE_240X240;
                else if (size_str == "320x240") size = FRAMESIZE_QVGA;
                else if (size_str == "400x296") size = FRAMESIZE_CIF;
                else if (size_str == "640x480") size = FRAMESIZE_VGA;
                else if (size_str == "800x600") size = FRAMESIZE_SVGA;
                
                bool success = camera_manager.SetFrameSize(size);
                return success ? "帧大小设置成功" : "帧大小设置失败";
            }
            return "摄像头未初始化";
        });

        mcp_server.AddTool("self.camera.set_pixel_format", "设置像素格式", PropertyList(std::vector<Property>{
            Property("format", kPropertyTypeString, "RGB565")
        }), [this](const PropertyList& properties) -> ReturnValue {
            auto& camera_manager = CameraManager::GetInstance();
            if (camera_manager.IsInitialized()) {
                std::string format_str = properties["format"].value<std::string>();
                pixformat_t format = PIXFORMAT_RGB565; // 默认值
                
                if (format_str == "RGB565") format = PIXFORMAT_RGB565;
                else if (format_str == "YUV422") format = PIXFORMAT_YUV422;
                else if (format_str == "GRAYSCALE") format = PIXFORMAT_GRAYSCALE;
                else if (format_str == "JPEG") format = PIXFORMAT_JPEG;
                
                bool success = camera_manager.SetPixelFormat(format);
                return success ? "像素格式设置成功" : "像素格式设置失败";
            }
            return "摄像头未初始化";
        });

        mcp_server.AddTool("self.camera.set_jpeg_quality", "设置JPEG质量", PropertyList(std::vector<Property>{
            Property("quality", kPropertyTypeInteger, 12, 1, 63)
        }), [this](const PropertyList& properties) -> ReturnValue {
            auto& camera_manager = CameraManager::GetInstance();
            if (camera_manager.IsInitialized()) {
                int quality = properties["quality"].value<int>();
                bool success = camera_manager.SetJpegQuality(quality);
                return success ? "JPEG质量设置成功" : "JPEG质量设置失败";
            }
            return "摄像头未初始化";
        });

        ESP_LOGI(TAG, "MCP工具初始化完成");
    }

public:
    ElegooRobotCar() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeRobotController();
        InitializeCamera();
        InitializeTools();
        InitializeWebServer();
        GetBacklight()->RestoreBrightness();
    }

    ~ElegooRobotCar() {
        if (robot_controller_) {
            robot_controller_->Shutdown();
            delete robot_controller_;
            robot_controller_ = nullptr;
        }
        // CameraManager使用单例模式，不需要手动删除
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                               AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                                               AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
                                               AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    } 

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        auto& camera_manager = CameraManager::GetInstance();
        return camera_manager.IsInitialized() ? &camera_manager : nullptr;
    }
};

DECLARE_BOARD(ElegooRobotCar);
