#pragma once

#include "camera.h"
#include "esp_camera.h"
#include <string>
#include <thread>
#include <lvgl.h>

class CameraManager : public Camera {
public:
    // 单例模式：获取实例
    static CameraManager& GetInstance();
    
    // 初始化方法（替代构造函数）
    bool Initialize(const camera_config_t& config);
    
    // 禁用拷贝构造函数和赋值操作符
    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    // 基础摄像头功能（继承自Camera类）
    void SetExplainUrl(const std::string& url, const std::string& token) override;
    bool Capture() override;
    bool SetHMirror(bool enable) override;
    bool SetVFlip(bool enable) override;
    std::string Explain(const std::string& question) override;

    // 简单的帧获取和返回方法
    camera_fb_t* GetFrame();
    void ReturnFrame(camera_fb_t* fb);

    // 视频流控制（简化版）
    bool StartStreaming();
    bool StopStreaming();

    // 摄像头参数设置
    bool SetFrameSize(framesize_t size);
    bool SetPixelFormat(pixformat_t format);
    bool SetJpegQuality(int quality);

    // 获取摄像头参数
    framesize_t GetFrameSize() const;
    pixformat_t GetPixelFormat() const;
    int GetJpegQuality() const;

    // 统计信息
    float GetFrameRate() const;
    void ResetStatistics();
    
    // 检查是否已初始化
    bool IsInitialized() const { return initialized_; }

private:
    // 私有构造函数和析构函数
    CameraManager();
    ~CameraManager();
    
    // 静态实例
    static CameraManager* instance_;
    
    // 初始化状态
    bool initialized_;
    
    std::string explain_url_;
    std::string explain_token_;
    camera_fb_t* fb_ = nullptr;
    lv_img_dsc_t preview_image_;
    std::thread encoder_thread_;
    
    // 流控制和统计
    bool streaming_enabled_;
    uint32_t frames_captured_;
    uint32_t frames_dropped_;
    uint64_t last_frame_time_;

    // JPEG编码相关结构
    struct JpegChunk {
        uint8_t* data;
        size_t len;
    };
};