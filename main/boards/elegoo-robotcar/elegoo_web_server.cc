#include "elegoo_web_server.h"
#include "camera_manager.h"
#include "config.h"
#include <esp_log.h>
#include <esp_camera.h>

static const char* TAG = "ElegooWebServer";

// 静态HTML内容 - 内存优化版本
static const char* html_content = 
    "<!DOCTYPE html><html><head><title>Elegoo Robot</title></head>"
    "<body><h1>Elegoo Robot Camera</h1>"
    "<img src='/Stream' style='max-width:100%;height:auto;'></body></html>";

ElegooWebServer::ElegooWebServer(CameraManager* camera_manager) 
    : server_(nullptr), camera_manager_(camera_manager) {
}

ElegooWebServer::~ElegooWebServer() {
    Stop();
}

bool ElegooWebServer::Start() {
    if (server_) {
        ESP_LOGW(TAG, "服务器已在运行");
        return true;
    }

    // 内存优化的服务器配置
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = WEB_SERVER_STACK_SIZE;
    config.task_priority = 3;
    config.max_open_sockets = 2;  // 限制最大连接数
    config.max_uri_handlers = 2;  // 限制URI处理器数量
    config.keep_alive_enable = false;  // 禁用keep-alive减少内存
    config.lru_purge_enable = true;    // 启用LRU清理

    esp_err_t ret = httpd_start(&server_, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动HTTP服务器失败: %s", esp_err_to_name(ret));
        return false;
    }

    RegisterHandlers();
    ESP_LOGI(TAG, "Web服务器启动成功，端口: %d", config.server_port);
    return true;
}

void ElegooWebServer::Stop() {
    if (server_) {
        httpd_stop(server_);
        server_ = nullptr;
        ESP_LOGI(TAG, "Web服务器已停止");
    }
}

void ElegooWebServer::RegisterHandlers() {
    // 根路径处理器
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = HandleRoot,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &root_uri);

    // 视频流处理器
    httpd_uri_t stream_uri = {
        .uri = "/Stream",
        .method = HTTP_GET,
        .handler = [](httpd_req_t* req) -> esp_err_t {
            ElegooWebServer* server = static_cast<ElegooWebServer*>(req->user_ctx);
            return server->HandleStream(req);
        },
        .user_ctx = this
    };
    httpd_register_uri_handler(server_, &stream_uri);
}

esp_err_t ElegooWebServer::HandleRoot(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_content, HTTPD_RESP_USE_STRLEN);
}

esp_err_t ElegooWebServer::HandleStream(httpd_req_t* req) {
    esp_err_t res = ESP_OK;
    
    // 设置MJPEG流响应头
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (!camera_manager_) {
        ESP_LOGE(TAG, "摄像头管理器未设置");
        return ESP_FAIL;
    }

    while (true) {
        camera_fb_t* fb = camera_manager_->GetFrame();
        if (!fb) {
            ESP_LOGW(TAG, "获取摄像头帧失败");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 发送MJPEG帧边界
        res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 12);
        if (res != ESP_OK) {
            camera_manager_->ReturnFrame(fb);
            break;
        }

        // 发送内容类型和长度
        char content_header[64];
        int header_len = snprintf(content_header, sizeof(content_header),
                                "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                (unsigned)fb->len);
        res = httpd_resp_send_chunk(req, content_header, header_len);
        if (res != ESP_OK) {
            camera_manager_->ReturnFrame(fb);
            break;
        }

        // 发送图像数据
        res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        camera_manager_->ReturnFrame(fb);
        
        if (res != ESP_OK) break;

        // 短暂延迟以控制帧率
        vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
    }

    return res;
}


extern "C" void elegoo_web_server_init_with_camera(void* camera_manager) {
    static ElegooWebServer* server = nullptr;
    
    // 如果已经初始化，直接返回成功
    if (server && server->IsRunning()) {
        ESP_LOGI(TAG, "Web服务器初始化成功");
        return;
    }
    
    // 创建新的服务器实例
    if (!server) {
        server = new ElegooWebServer(static_cast<CameraManager*>(camera_manager));
    } else {
        // 更新摄像头管理器
        server->SetCameraManager(static_cast<CameraManager*>(camera_manager));
    }
    
    if (server && server->Start()) {
        ESP_LOGI(TAG, "Web服务器初始化成功");
        
    }else{
        ESP_LOGE(TAG, "Web服务器初始化失败");
    }
    
}
