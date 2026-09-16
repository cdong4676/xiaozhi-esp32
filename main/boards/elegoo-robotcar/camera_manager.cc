#include "camera_manager.h"
#include "display.h"
#include "display/lcd_display.h"
#include "display/lvgl_display/lvgl_image.h"
#include "board.h"
#include "system_info.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <img_converters.h>
#include <esp_timer.h>

#define TAG "CameraManager"

CameraManager::CameraManager() 
    : initialized_(false), streaming_enabled_(false), frames_captured_(0), last_frame_time_(0) {
    // 构造函数只初始化基本成员变量
    fb_ = nullptr;
    ESP_LOGI(TAG, "CameraManager instance created");
}

bool CameraManager::Initialize(const camera_config_t& config) {
    if (initialized_) {
        ESP_LOGW(TAG, "CameraManager already initialized");
        return true;
    }
    
    // 初始化摄像头
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get(); // 获取摄像头型号
    if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);  // 这里控制摄像头镜像 写1镜像 写0不镜像
    }
    s->set_vflip(s, 1);  // 这里控制摄像头垂直镜像 写1镜像 写0不镜像

    // 初始化预览图片的内存
    initialized_ = true;
    ESP_LOGI(TAG, "Camera manager initialized successfully");
    return true;
}

CameraManager::~CameraManager() {
    StopStreaming();
    
    if (fb_) {
        esp_camera_fb_return(fb_);
        fb_ = nullptr;
    }
    
    if (initialized_) {
        esp_camera_deinit();
        initialized_ = false;
    }
    
    ESP_LOGI(TAG, "Camera manager destroyed");
}



void CameraManager::SetExplainUrl(const std::string& url, const std::string& token) {
    explain_url_ = url;
    explain_token_ = token;
}



bool CameraManager::SetHMirror(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }
    
    esp_err_t err = s->set_hmirror(s, enabled);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set horizontal mirror: %d", err);
        return false;
    }
    
    ESP_LOGI(TAG, "Camera horizontal mirror set to: %s", enabled ? "enabled" : "disabled");
    return true;
}

bool CameraManager::SetVFlip(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }
    
    esp_err_t err = s->set_vflip(s, enabled);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set vertical flip: %d", err);
        return false;
    }
    
    ESP_LOGI(TAG, "Camera vertical flip set to: %s", enabled ? "enabled" : "disabled");
    return true;
}

bool CameraManager::StartStreaming() {
    if (streaming_enabled_) {
        ESP_LOGW(TAG, "Streaming already started");
        return true;
    }
    
    streaming_enabled_ = true;
    ResetStatistics();
    
    ESP_LOGI(TAG, "Video streaming started");
    return true;
}

bool CameraManager::StopStreaming() {
    if (!streaming_enabled_) {
        return true;
    }
    
    streaming_enabled_ = false;
    
    ESP_LOGI(TAG, "Video streaming stopped");
    return true;
}

camera_fb_t* CameraManager::GetFrame() {
    camera_fb_t* fb = esp_camera_fb_get();
    
    if (fb == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera frame");
        return nullptr;
    }
    
    // 验证帧数据完整性
    if (fb->buf == nullptr || fb->len == 0) {
        ESP_LOGE(TAG, "Invalid frame data: buf=%p, len=%d", fb->buf, fb->len);
        esp_camera_fb_return(fb);
        return nullptr;
    }
    
    return fb;
}

void CameraManager::ReturnFrame(camera_fb_t* fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    } else {
        ESP_LOGW(TAG, "ReturnFrame: 尝试返回NULL帧指针");
    }
}

float CameraManager::GetFrameRate() const {
    if (frames_captured_ == 0 || last_frame_time_ == 0) {
        return 0.0f;
    }
    
    uint64_t current_time = esp_timer_get_time();
    uint64_t elapsed_time = current_time - last_frame_time_;
    
    if (elapsed_time == 0) {
        return 0.0f;
    }
    
    return (float)frames_captured_ / (elapsed_time / 1000000.0f);
}

bool CameraManager::SetFrameSize(framesize_t size) {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }
    
    esp_err_t err = s->set_framesize(s, size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set frame size: %d", err);
        return false;
    }
    
    ESP_LOGI(TAG, "Camera frame size set to: %d", size);
    return true;
}

bool CameraManager::SetPixelFormat(pixformat_t format) {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }
    
    esp_err_t err = s->set_pixformat(s, format);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pixel format: %d", err);
        return false;
    }
    
    ESP_LOGI(TAG, "Camera pixel format set to: %d", format);
    return true;
}

bool CameraManager::SetJpegQuality(int quality) {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }
    
    esp_err_t err = s->set_quality(s, quality);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set JPEG quality: %d", err);
        return false;
    }
    
    ESP_LOGI(TAG, "Camera JPEG quality set to: %d", quality);
    return true;
}

framesize_t CameraManager::GetFrameSize() const {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        return FRAMESIZE_INVALID;
    }
    return s->status.framesize;
}

pixformat_t CameraManager::GetPixelFormat() const {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        return PIXFORMAT_RGB565;
    }
    return s->pixformat;
}

int CameraManager::GetJpegQuality() const {
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr) {
        return -1;
    }
    return s->status.quality;
}

void CameraManager::ResetStatistics() {
    frames_captured_ = 0;
    last_frame_time_ = esp_timer_get_time();
}

bool CameraManager::Capture() {
   int frames_to_get = 2;
    // Try to get a stable frame
    for (int i = 0; i < frames_to_get; i++) {
        if (fb_ != nullptr) {
            esp_camera_fb_return(fb_);
        }
        fb_ = esp_camera_fb_get();
        if (fb_ == nullptr) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }
    }

    auto display = Board::GetInstance().GetDisplay();
    if (display == nullptr || fb_->width == 0 || fb_->height == 0) {
        return true;
    }

    const size_t preview_size = fb_->width * fb_->height * sizeof(uint16_t);
    auto* preview_data = static_cast<uint8_t*>(heap_caps_malloc(preview_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (preview_data == nullptr) {
        ESP_LOGW(TAG, "Failed to allocate preview image buffer");
        return true;
    }

    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor != nullptr && sensor->pixformat == PIXFORMAT_JPEG) {
        if (!jpg2rgb565(fb_->buf, fb_->len, preview_data, JPG_SCALE_NONE)) {
            ESP_LOGE(TAG, "Failed to decode JPEG for preview");
            heap_caps_free(preview_data);
            return true;
        }
    } else {
        if (fb_->len < preview_size) {
            ESP_LOGE(TAG, "RGB565 frame is too small for preview: %zu < %zu", fb_->len, preview_size);
            heap_caps_free(preview_data);
            return true;
        }
        auto* source = reinterpret_cast<const uint16_t*>(fb_->buf);
        auto* destination = reinterpret_cast<uint16_t*>(preview_data);
        for (size_t i = 0; i < preview_size / sizeof(uint16_t); ++i) {
            destination[i] = __builtin_bswap16(source[i]);
        }
    }

    auto* lcd_display = static_cast<LcdDisplay*>(display);
    lcd_display->SetPreviewImage(std::make_unique<LvglAllocatedImage>(
        preview_data, preview_size, fb_->width, fb_->height, fb_->width * sizeof(uint16_t), LV_COLOR_FORMAT_RGB565));
    return true;
}

std::string CameraManager::Explain(const std::string& question) {
    if (explain_url_.empty()) {
        return "{\"success\": false, \"message\": \"Image explain URL or token is not set\"}";
    }

    if (fb_ == nullptr || fb_->buf == nullptr || fb_->len == 0) {
        return "{\"success\": false, \"message\": \"No valid image data available\"}";
    }

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);
    std::string boundary = "----ESP32_CAMERA_BOUNDARY";

    // 配置HTTP客户端，使用分块传输编码
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");

    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        return "{\"success\": false, \"message\": \"Failed to connect to explain URL\"}";
    }

    // 第一部分：question字段
    std::string question_field = "--" + boundary + "\r\n"
                                "Content-Disposition: form-data; name=\"question\"\r\n"
                                "\r\n" + question + "\r\n";
    http->Write(question_field.c_str(), question_field.size());

    // 第二部分：文件字段头部
    std::string file_header = "--" + boundary + "\r\n"
                             "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n"
                             "Content-Type: image/jpeg\r\n"
                             "\r\n";
    http->Write(file_header.c_str(), file_header.size());

    // 第三部分：图像数据
    size_t total_sent = 0;
    sensor_t *s = esp_camera_sensor_get();
    
    if (s != nullptr && s->pixformat == PIXFORMAT_JPEG) {
        // 直接发送JPEG数据
        http->Write((const char*)fb_->buf, fb_->len);
        total_sent = fb_->len;
    } else {
        // RGB格式需要转换为JPEG，使用回调直接写入HTTP
        bool conversion_success = frame2jpg_cb(fb_, 80, 
            [](void* arg, size_t index, const void* data, size_t len) -> unsigned int {
                auto http_ptr = static_cast<Http*>(arg);
                http_ptr->Write((const char*)data, len);
                return len;
            }, http.get());
        
        if (!conversion_success) {
            http->Close();
            return "{\"success\": false, \"message\": \"Failed to convert image to JPEG\"}";
        }
        total_sent = fb_->len; // 近似值
    }

    // 第四部分：multipart结束
    std::string multipart_footer = "\r\n--" + boundary + "--\r\n";
    http->Write(multipart_footer.c_str(), multipart_footer.size());

    // 结束分块传输
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        http->Close();
        return "{\"success\": false, \"message\": \"Failed to upload photo\"}";
    }

    std::string result = http->ReadAll();
    http->Close();

    esp_camera_fb_return(fb_);
    ESP_LOGI(TAG, "Explain image size=%dx%d, sent size=%d, question=%s",
        fb_->width, fb_->height, total_sent, question.c_str());
    
    return result;
}
