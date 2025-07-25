#include "camera_manager.h"
#include "mcp_server.h"
#include "display.h"
#include "board.h"
#include "system_info.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <img_converters.h>
#include <cstring>
#include <esp_timer.h>
#include <thread>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define TAG "CameraManager"

// 静态实例初始化
CameraManager* CameraManager::instance_ = nullptr;

CameraManager& CameraManager::GetInstance() {
    if (instance_ == nullptr) {
        instance_ = new CameraManager();
    }
    return *instance_;
}

CameraManager::CameraManager() 
    : initialized_(false), streaming_enabled_(false), frames_captured_(0), frames_dropped_(0), last_frame_time_(0) {
    // 构造函数只初始化基本成员变量
    fb_ = nullptr;
    memset(&preview_image_, 0, sizeof(preview_image_));
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

    // 初始化预览图像缓冲区
    sensor_t* s = esp_camera_sensor_get();
    if (s != nullptr) {
        if (s->id.PID == OV2640_PID) {
            s->set_framesize(s, FRAMESIZE_240X240);
            s->set_vflip(s, 1);
            s->set_hmirror(s, 1);
        }
    }

    // 获取帧缓冲区信息
    camera_fb_t* test_fb = esp_camera_fb_get();
    if (test_fb != nullptr) {
        size_t preview_size = test_fb->width * test_fb->height * 2; // RGB565
        preview_image_.data = (uint8_t*)heap_caps_aligned_alloc(16, preview_size, MALLOC_CAP_SPIRAM);
        if (preview_image_.data != nullptr) {
            preview_image_.data_size = preview_size;
            preview_image_.header.w = test_fb->width;
            preview_image_.header.h = test_fb->height;
            preview_image_.header.cf = LV_IMG_CF_TRUE_COLOR;
        }
        esp_camera_fb_return(test_fb);
    }

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
    
    if (preview_image_.data) {
        heap_caps_free((void*)preview_image_.data);
        preview_image_.data = nullptr;
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

bool CameraManager::Capture() {
    if (encoder_thread_.joinable()) {
        encoder_thread_.join();
    }

    int frames_to_get = 2;
    // 尝试获取稳定的帧
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

    frames_captured_++;
    last_frame_time_ = esp_timer_get_time();

    // 如果预览图片buffer为空，则跳过预览
    if (preview_image_.data_size == 0) {
        ESP_LOGW(TAG, "Skip preview because of unsupported frame size");
        return true;
    }
    
    if (preview_image_.data == nullptr) {
        ESP_LOGE(TAG, "Preview image data is not initialized");
        return true;
    }
    
    // 显示预览图片
    auto display = Board::GetInstance().GetDisplay();
    if (display != nullptr) {
        auto src = (uint16_t*)fb_->buf;
        auto dst = (uint16_t*)preview_image_.data;
        size_t pixel_count = fb_->len / 2;
        for (size_t i = 0; i < pixel_count; i++) {
            // 交换每个16位字内的字节
            dst[i] = __builtin_bswap16(src[i]);
        }
        display->SetPreviewImage(&preview_image_);
    }
    
    return true;
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
    return esp_camera_fb_get();
}

void CameraManager::ReturnFrame(camera_fb_t* fb) {
    if (fb) {
        esp_camera_fb_return(fb);
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
    frames_dropped_ = 0;
    last_frame_time_ = esp_timer_get_time();
}

std::string CameraManager::Explain(const std::string& question) {
    if (explain_url_.empty()) {
        return "{\"success\": false, \"message\": \"Image explain URL or token is not set\"}";
    }

    // 创建局部的JPEG队列
    QueueHandle_t jpeg_queue = xQueueCreate(40, sizeof(JpegChunk));
    if (jpeg_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create JPEG queue");
        return "{\"success\": false, \"message\": \"Failed to create JPEG queue\"}";
    }

    // 生成JPEG编码线程
    encoder_thread_ = std::thread([this, jpeg_queue]() {
        frame2jpg_cb(fb_, 80, [](void* arg, size_t index, const void* data, size_t len) -> unsigned int {
            auto jpeg_queue = (QueueHandle_t)arg;
            JpegChunk chunk = {
                .data = (uint8_t*)heap_caps_aligned_alloc(16, len, MALLOC_CAP_SPIRAM),
                .len = len
            };
            memcpy(chunk.data, data, len);
            xQueueSend(jpeg_queue, &chunk, portMAX_DELAY);
            return len;
        }, jpeg_queue);
    });

    auto http = Board::GetInstance().CreateHttp();
    // 构造multipart/form-data请求体
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
        // 清理队列
        encoder_thread_.join();
        JpegChunk chunk;
        while (xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY) == pdPASS) {
            if (chunk.data != nullptr) {
                heap_caps_free(chunk.data);
            } else {
                break;
            }
        }
        vQueueDelete(jpeg_queue);
        return "{\"success\": false, \"message\": \"Failed to connect to explain URL\"}";
    }
    
    {
        // 第一块：question字段
        std::string question_field;
        question_field += "--" + boundary + "\r\n";
        question_field += "Content-Disposition: form-data; name=\"question\"\r\n";
        question_field += "\r\n";
        question_field += question + "\r\n";
        http->Write(question_field.c_str(), question_field.size());
    }
    {
        // 第二块：文件字段头部
        std::string file_header;
        file_header += "--" + boundary + "\r\n";
        file_header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
        file_header += "Content-Type: image/jpeg\r\n";
        file_header += "\r\n";
        http->Write(file_header.c_str(), file_header.size());
    }

    // 第三块：JPEG数据
    size_t total_sent = 0;
    while (true) {
        JpegChunk chunk;
        if (xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to receive JPEG chunk");
            break;
        }
        if (chunk.data == nullptr) {
            break; // 最后一个块
        }
        http->Write((const char*)chunk.data, chunk.len);
        total_sent += chunk.len;
        heap_caps_free(chunk.data);
    }
    
    // 等待编码线程完成
    encoder_thread_.join();
    // 清理队列
    vQueueDelete(jpeg_queue);

    {
        // 第四块：multipart尾部
        std::string multipart_footer;
        multipart_footer += "\r\n--" + boundary + "--\r\n";
        http->Write(multipart_footer.c_str(), multipart_footer.size());
    }
    // 结束块
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        return "{\"success\": false, \"message\": \"Failed to upload photo\"}";
    }

    std::string result = http->ReadAll();
    http->Close();

    // 获取剩余任务栈大小
    size_t remain_stack_size = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG, "Explain image size=%dx%d, compressed size=%d, remain stack size=%d, question=%s\n%s",
        fb_->width, fb_->height, total_sent, remain_stack_size, question.c_str(), result.c_str());
    return result;
}