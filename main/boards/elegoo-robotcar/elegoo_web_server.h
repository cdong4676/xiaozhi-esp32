#pragma once

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_err.h>
#include <string>

// 前向声明
class CameraManager;

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化Elegoo Web服务器（带摄像头管理器）
 * @param camera_manager 摄像头管理器实例
 * @return ESP_OK 成功，其他值表示失败
 */
void elegoo_web_server_init_with_camera(void* camera_manager);

#ifdef __cplusplus
}
#endif

// C++类定义
#ifdef __cplusplus

/**
 * @brief Elegoo机器人Web服务器类 - 内存优化版本
 * 
 * 提供HTTP视频流服务，专为ESP32内存限制优化
 */
class ElegooWebServer {
public:
    /**
     * @brief 构造函数
     * @param camera_manager 摄像头管理器实例
     */
    ElegooWebServer(CameraManager* camera_manager = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~ElegooWebServer();

    /**
     * @brief 启动Web服务器
     * @return true 启动成功，false 启动失败
     */
    bool Start();

    /**
     * @brief 停止Web服务器
     */
    void Stop();

    /**
     * @brief 检查服务器是否运行
     * @return true 运行中，false 已停止
     */
    bool IsRunning() const { return server_ != nullptr; }

    /**
     * @brief 设置摄像头管理器
     * @param camera_manager 摄像头管理器实例
     */
    void SetCameraManager(CameraManager* camera_manager) { camera_manager_ = camera_manager; }

private:

    /**
     * @brief 注册HTTP处理器
     */
    void RegisterHandlers();

    /**
     * @brief 根路径处理器
     */
    static esp_err_t HandleRoot(httpd_req_t* req);

    /**
     * @brief 视频流处理器
     */
    esp_err_t HandleStream(httpd_req_t* req);

    httpd_handle_t server_ = nullptr;
    CameraManager* camera_manager_ = nullptr;
};
#endif
