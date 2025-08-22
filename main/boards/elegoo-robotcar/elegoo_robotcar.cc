#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <wifi_station.h>
#include <esp_wifi.h>
#include <esp_event.h>

#include "codecs/no_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "display/lcd_display.h"
#include "otto_emoji_display.h"
#include "system_reset.h"
#include "wifi_board.h"
#include "camera_manager.h"
#include "elegoo_web_server.h"
#include "mcp_server.h"
#include "elegoo_robot_controller.h"

#define TAG "ElegooRobotCar"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

class ElegooRobotCar : public WifiBoard {
private:
    // 核心组件
    LcdDisplay* display_;
    Button boot_button_;
    ElegooRobotController* robot_controller_;
    CameraManager* camera_manager_;
    
    // 状态标志
    bool web_server_initialized_;

    // WiFi事件处理器
    static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data) {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
            auto* instance = static_cast<ElegooRobotCar*>(arg);
            instance->OnWifiConnected();
        }
    }

    // WiFi连接成功后的处理
    void OnWifiConnected() {
        ESP_LOGI(TAG, "WiFi连接成功，初始化服务器");
        
        if (!web_server_initialized_) {
            //elegoo_web_server_init_with_camera(camera_manager_);
        }

        if (robot_controller_) {
            robot_controller_->StartNetworkServerWhenReady();
        }
    }
    // 硬件初始化方法
    void InitializeHardware() {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeCamera();
        InitializeRobotController();
    }

    // 服务初始化方法  
    void InitializeServices() {
        InitializeTools();
        InitializeWebServer();
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {
            .mosi_io_num = DISPLAY_MOSI_PIN,
            .miso_io_num = GPIO_NUM_NC,
            .sclk_io_num = DISPLAY_CLK_PIN,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t)
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 配置SPI接口
        esp_lcd_panel_io_spi_config_t io_config = {
            .cs_gpio_num = DISPLAY_CS_PIN,
            .dc_gpio_num = DISPLAY_DC_PIN,
            .spi_mode = 0,
            .pclk_hz = CAMERA_PCLK_HZ,
            .trans_queue_depth = 10,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 配置LCD面板
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = DISPLAY_RST_PIN,
            .rgb_ele_order = DISPLAY_RGB_ORDER,
            .bits_per_pixel = 16
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        // 初始化面板
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);
        
        // 创建显示对象
        display_ = new OttoEmojiDisplay(
            panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, 
            DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
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

    void InitializeRobotController() {
        robot_controller_ = new ElegooRobotController();
        
        bool success = robot_controller_->Initialize(
            ECHO_UART_PORT_NUM, ECHO_UART_BAUD_RATE, 
            UART_ECHO_TXD, UART_ECHO_RXD, 
            UART_ECHO_RTS, UART_ECHO_CTS, BUF_SIZE
        );
        
        if (!success) {
            ESP_LOGE(TAG, "机器人控制器初始化失败");
            delete robot_controller_;
            robot_controller_ = nullptr;
        } else {
            ESP_LOGI(TAG, "机器人控制器初始化完成");
            
            // 将摄像头管理器传递给机器人控制器
            if (camera_manager_) {
                robot_controller_->SetCameraManager(camera_manager_);
                ESP_LOGI(TAG, "摄像头管理器已设置到机器人控制器");
            } else {
                ESP_LOGW(TAG, "摄像头管理器未初始化，无法设置到机器人控制器");
            }
        }
    }

    void InitializeCamera() {
        ESP_LOGI(TAG, "初始化摄像头...");
        
        camera_config_t config = {};
        config.pin_d0 = CAMERA_D0;
        config.pin_d1 = CAMERA_D1;
        config.pin_d2 = CAMERA_D2;
        config.pin_d3 = CAMERA_D3;
        config.pin_d4 = CAMERA_D4;
        config.pin_d5 = CAMERA_D5;
        config.pin_d6 = CAMERA_D6;
        config.pin_d7 = CAMERA_D7;
        config.pin_xclk = CAMERA_XCLK;
        config.pin_pclk = CAMERA_PCLK;
        config.pin_vsync = CAMERA_VSYNC;
        config.pin_href = CAMERA_HSYNC;
        config.pin_sccb_sda = CAMERA_SIOD;
        config.pin_sccb_scl = CAMERA_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PWDN;
        config.pin_reset = CAMERA_RESET;
        config.xclk_freq_hz = CAMERA_XCLK_FREQ;
        config.ledc_channel = LEDC_CHANNEL;
        config.ledc_timer = LEDC_TIMER;
        config.pixel_format = PIXFORMAT_JPEG;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        
        camera_manager_ = new CameraManager();
        if (!camera_manager_->Initialize(config)) {
            ESP_LOGE(TAG, "摄像头初始化失败");
            delete camera_manager_;
            camera_manager_ = nullptr;
        } else {
            ESP_LOGI(TAG, "摄像头初始化成功");
        }
    }

    void InitializeWebServer() {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
                                                 &wifi_event_handler, this));
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        
        // 底盘控制工具
        RegisterChassisTools(mcp_server);
        
        // 摄像头控制工具
        RegisterCameraTools(mcp_server);
        
        ESP_LOGI(TAG, "MCP工具初始化完成");
    }

private:
    void RegisterChassisTools(McpServer& mcp_server) {
        // 方向移动控制
        mcp_server.AddTool("self.chassis.move_by_direction", 
            "根据方向和速度移动。direction:移动方向(0=停止,1=前进,2=后退,3=左转,4=右转,5=左上,6=左下,7=右上,8=右下);speed:速度(0-255)", 

            PropertyList({
                Property("direction", kPropertyTypeInteger, 1, 0, 9),
                Property("speed", kPropertyTypeInteger, 200, 0, 250)
            }), 
            [this](const PropertyList& properties) -> ReturnValue {
                int direction = properties["direction"].value<int>();
                int speed = properties["speed"].value<int>();
                robot_controller_->ExecuteMoveCommand(direction, speed);
                return true;
            });

        // 停止和待机
        mcp_server.AddTool("self.chassis.stop", "停止", PropertyList(), 
            [this](const PropertyList&) -> ReturnValue {
                robot_controller_->ExecuteStandby();
                return true;
            });

        mcp_server.AddTool("self.chassis.standby", "待机", PropertyList(), 
            [this](const PropertyList&) -> ReturnValue {
                robot_controller_->ExecuteStandby();
                return true;
            });

        // 模式设置
        mcp_server.AddTool("self.chassis.set_mode", "设置功能模式。mode:(1=巡线,2=避障,3=跟随)", 
            PropertyList({Property("mode", kPropertyTypeInteger, 1, 1, 3)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                int mode = properties["mode"].value<int>();
                robot_controller_->ExecuteModeChangeCommand(mode);
                return true;
            });

        // 电机控制
        mcp_server.AddTool("self.chassis.control_motors", 
            "控制电机。select:选择要控制的电机(0=同时控制左右电机,1=控制左电机,2=控制右电机);speed:速度(0-255);direction:方向(1=顺时针,2=逆时针)", 
            PropertyList({
                Property("select", kPropertyTypeInteger, 0, 0, 2),
                Property("speed", kPropertyTypeInteger, 200, 0, 255),
                Property("direction", kPropertyTypeInteger, 1, 1, 2)
            }), 
            [this](const PropertyList& properties) -> ReturnValue {
                int select = properties["select"].value<int>();
                int speed = properties["speed"].value<int>();
                int direction = properties["direction"].value<int>();
                robot_controller_->ExecuteMotorControl(select, speed, direction);
                return true;
            });

        // 速度设置
        mcp_server.AddTool("self.chassis.set_speed", "设置移动速度", 
            PropertyList({Property("speed", kPropertyTypeInteger, 200, 0, 255)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                int speed = properties["speed"].value<int>();
                robot_controller_->ExecuteSetMoveSpeed(speed);
                return true;
            });

        // 舵机控制
        mcp_server.AddTool("self.chassis.control_servo", "控制云台舵机角度。degree:舵机角度(0-180)", 
            PropertyList({Property("degree", kPropertyTypeInteger, 90, 0, 180)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                int degree = std::max(0, std::min(180, properties["degree"].value<int>()));
                robot_controller_->ExecuteServoControl(degree);
                return true;
            });
    }

    void RegisterCameraTools(McpServer& mcp_server) {
        // 摄像头镜像和翻转
        mcp_server.AddTool("self.camera.set_hmirror", "设置水平镜像", 
            PropertyList({Property("enable", kPropertyTypeBoolean, false)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                if (camera_manager_ && camera_manager_->IsInitialized()) {
                    bool enable = properties["enable"].value<bool>();
                    camera_manager_->SetHMirror(enable);
                    return "设置成功";
                }
                return "摄像头未初始化";
            });

        mcp_server.AddTool("self.camera.set_vflip", "设置垂直翻转", 
            PropertyList({Property("enable", kPropertyTypeBoolean, false)}), 
            [this](const PropertyList& properties) -> ReturnValue {
                if (camera_manager_ && camera_manager_->IsInitialized()) {
                    bool enable = properties["enable"].value<bool>();
                    camera_manager_->SetVFlip(enable);
                    return "设置成功";
                }
                return "摄像头未初始化";
            });

        // 拍照和AI分析
        mcp_server.AddTool("self.camera.capture_and_analyze", "拍照并识别物体", 
            PropertyList({Property("question", kPropertyTypeString, "请描述你看到了什么?")}), 
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG, "拍照并识别物体");
                
                // 检查摄像头管理器是否存在
                if (!camera_manager_) {
                    ESP_LOGE(TAG, "摄像头管理器未初始化");
                    return "{\"success\": false, \"message\": \"Camera manager not initialized\"}";
                }
                
                // 检查摄像头是否已初始化
                if (!camera_manager_->IsInitialized()) {
                    ESP_LOGE(TAG, "摄像头未初始化");
                    return "{\"success\": false, \"message\": \"Camera not initialized\"}";
                }
                
                // 尝试拍照
                if (!camera_manager_->Capture()) {
                    ESP_LOGE(TAG, "拍照失败");
                    return "{\"success\": false, \"message\": \"Failed to capture image\"}";
                }
                
                // 获取问题参数
                std::string question = "识别并描述图片中的主体";
                try {
                    question = properties["question"].value<std::string>();
                } catch (const std::runtime_error&) {
                    ESP_LOGW(TAG, "使用默认问题");
                }
                
                // 进行AI分析
                std::string result = camera_manager_->Explain(question);
                ESP_LOGI(TAG, "AI分析完成");
                return result;
            });

        // 帧大小设置
        mcp_server.AddTool("self.camera.set_frame_size", "设置帧大小", 
            PropertyList({Property("size", kPropertyTypeString, "240x240")}), 
            [this](const PropertyList& properties) -> ReturnValue {
                if (!camera_manager_ || !camera_manager_->IsInitialized()) {
                    return "摄像头未初始化";
                }
                
                std::string size_str = properties["size"].value<std::string>();
                framesize_t size = FRAMESIZE_240X240;
                
                if (size_str == "96x96") size = FRAMESIZE_96X96;
                else if (size_str == "160x120") size = FRAMESIZE_QQVGA;
                else if (size_str == "176x144") size = FRAMESIZE_QCIF;
                else if (size_str == "240x176") size = FRAMESIZE_HQVGA;
                else if (size_str == "240x240") size = FRAMESIZE_240X240;
                else if (size_str == "320x240") size = FRAMESIZE_QVGA;
                else if (size_str == "400x296") size = FRAMESIZE_CIF;
                else if (size_str == "640x480") size = FRAMESIZE_VGA;
                else if (size_str == "800x600") size = FRAMESIZE_SVGA;
                
                return camera_manager_->SetFrameSize(size) ? "帧大小设置成功" : "帧大小设置失败";
            });
    }

public:
    ElegooRobotCar() : boot_button_(BOOT_BUTTON_GPIO), camera_manager_(nullptr), web_server_initialized_(false) {
        InitializeHardware();
        InitializeServices();
        GetBacklight()->RestoreBrightness();
    }

    ~ElegooRobotCar() {
        if (robot_controller_) {
            robot_controller_->Shutdown();
            delete robot_controller_;
            robot_controller_ = nullptr;
        }
        
        if (camera_manager_) {
            delete camera_manager_;
            camera_manager_ = nullptr;
        }
    }

    // 重写基类虚函数
    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { 
        return display_; 
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        // CameraManager现在继承自Esp32Camera，可以返回camera_manager_
        return camera_manager_;
    }

};

DECLARE_BOARD(ElegooRobotCar);
