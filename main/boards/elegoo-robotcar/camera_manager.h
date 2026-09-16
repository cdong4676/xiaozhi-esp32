#pragma once

#include "camera.h"
#include "esp_camera.h"
#include <string>

class CameraManager : public Camera {
public:
    // 构造函数和析构函数
    CameraManager();
    virtual ~CameraManager();
    
    // 初始化方法
    bool Initialize(const camera_config_t& config);
    
    // 禁用拷贝构造函数和赋值操作符
    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    // 基础摄像头功能（继承自Camera类）
    virtual void SetExplainUrl(const std::string& url, const std::string& token) override;
    virtual bool Capture() override;
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual std::string Explain(const std::string& question) override;

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
    // 初始化状态
    bool initialized_;
    
    std::string explain_url_;
    std::string explain_token_;
    camera_fb_t* fb_ = nullptr;
    
    // 流控制和统计
    bool streaming_enabled_;
    uint32_t frames_captured_;
    uint64_t last_frame_time_;
};
