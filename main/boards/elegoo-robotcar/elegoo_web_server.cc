#include "elegoo_web_server.h"
#include "config.h"
#include "mcp_server.h"
#include "board.h"
#include "camera_manager.h"
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <cJSON.h>
#include <string>
#include <esp_camera.h>

// 定义MIN宏（如果未定义）
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define TAG "ElegooWebServer"

static httpd_handle_t server = nullptr;
static bool face_detection_enabled = false;

// 获取CameraManager实例的辅助函数
static CameraManager* GetCamera() {
    auto& camera_manager = CameraManager::GetInstance();
    return camera_manager.IsInitialized() ? &camera_manager : nullptr;
}

// HTML页面内容 - 分离为多个字符串以避免编译问题
static const char* html_header = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"    <meta charset=\"UTF-8\">\n"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
"    <title>Elegoo Robot Car Control</title>\n"
"    <style>\n"
"        * { margin: 0; padding: 0; box-sizing: border-box; }\n"
"        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%); color: white; min-height: 100vh; overflow-x: hidden; }\n"
"        .container { max-width: 1400px; margin: 0 auto; padding: 20px; }\n"
"        h1 { text-align: center; margin-bottom: 30px; font-size: 2.5em; text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5); color: #fff; }\n"
"        .main-layout { display: grid; grid-template-columns: 1fr 400px; gap: 30px; min-height: 80vh; }\n"
"        .video-section { background: rgba(255, 255, 255, 0.1); border-radius: 20px; padding: 20px; backdrop-filter: blur(10px); box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3); display: flex; flex-direction: column; }\n"
"        .video-container { flex: 1; display: flex; align-items: center; justify-content: center; background: #000; border-radius: 15px; overflow: hidden; position: relative; min-height: 400px; }\n"
"        .video-stream { max-width: 100%; max-height: 100%; object-fit: contain; border-radius: 15px; }\n"
"        .video-placeholder { color: #888; font-size: 1.2em; text-align: center; }\n"
"        .control-panel { background: rgba(255, 255, 255, 0.1); border-radius: 20px; padding: 20px; backdrop-filter: blur(10px); box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3); display: flex; flex-direction: column; gap: 20px; }\n"
"        .control-section { background: rgba(255, 255, 255, 0.1); border-radius: 15px; padding: 15px; }\n"
"        .control-section h3 { margin-bottom: 15px; color: #fff; border-bottom: 2px solid rgba(255, 255, 255, 0.3); padding-bottom: 8px; font-size: 1.1em; }\n"
"        .joystick-container { display: flex; justify-content: center; margin-bottom: 20px; }\n"
"        .joystick { width: 150px; height: 150px; border: 3px solid rgba(255, 255, 255, 0.3); border-radius: 50%; position: relative; background: radial-gradient(circle, rgba(255, 255, 255, 0.1) 0%, rgba(255, 255, 255, 0.05) 100%); cursor: pointer; }\n"
"        .joystick-knob { width: 40px; height: 40px; background: linear-gradient(45deg, #4CAF50, #45a049); border-radius: 50%; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3); transition: all 0.1s ease; }\n"
"        .mode-buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 20px; }\n"
"        .mode-btn { padding: 12px 16px; font-size: 14px; border: none; border-radius: 10px; cursor: pointer; transition: all 0.3s ease; font-weight: bold; text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.3); }\n"
"        .mode-btn.obstacle { background: linear-gradient(45deg, #ff9800, #f57c00); color: white; }\n"
"        .mode-btn.line { background: linear-gradient(45deg, #2196F3, #1976D2); color: white; }\n"
"        .mode-btn.follow { background: linear-gradient(45deg, #4CAF50, #45a049); color: white; }\n"
"        .mode-btn.standby { background: linear-gradient(45deg, #9E9E9E, #757575); color: white; }\n"
"        .mode-btn:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0, 0, 0, 0.3); }\n"
"        .mode-btn:active { transform: translateY(0); }\n"
"        .mode-btn.active { box-shadow: 0 0 20px rgba(255, 255, 255, 0.5); border: 2px solid rgba(255, 255, 255, 0.8); }\n"
"        .motor-btn { flex: 1; padding: 8px 4px; font-size: 16px; border: none; border-radius: 8px; cursor: pointer; background: linear-gradient(45deg, #607D8B, #455A64); color: white; font-weight: bold; transition: all 0.3s ease; }\n"
"        .motor-btn:hover { transform: translateY(-1px); box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3); background: linear-gradient(45deg, #78909C, #546E7A); }\n"
"        .motor-btn:active { transform: translateY(0); }\n"
"        .motor-speed-slider { width: 100%; height: 6px; border-radius: 3px; background: rgba(255, 255, 255, 0.3); outline: none; -webkit-appearance: none; margin: 8px 0; }\n"
"        .motor-speed-slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 16px; height: 16px; border-radius: 50%; background: #FF5722; cursor: pointer; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.3); }\n"
"        .motor-speed-slider::-moz-range-thumb { width: 16px; height: 16px; border-radius: 50%; background: #FF5722; cursor: pointer; border: none; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.3); }\n"
"        .sync-motors-btn { width: 100%; padding: 10px; font-size: 14px; border: none; border-radius: 10px; cursor: pointer; background: linear-gradient(45deg, #E91E63, #C2185B); color: white; font-weight: bold; transition: all 0.3s ease; }\n"
"        .sync-motors-btn:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0, 0, 0, 0.3); }\n"
"        .speed-control { margin-top: 15px; }\n"
"        .speed-control label { display: block; margin-bottom: 8px; font-weight: bold; font-size: 0.9em; }\n"
"        .speed-slider { width: 100%; height: 8px; border-radius: 5px; background: rgba(255, 255, 255, 0.3); outline: none; -webkit-appearance: none; }\n"
"        .speed-slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #4CAF50; cursor: pointer; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.3); }\n"
"        .speed-slider::-moz-range-thumb { width: 20px; height: 20px; border-radius: 50%; background: #4CAF50; cursor: pointer; border: none; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.3); }\n"
"        .status-display { background: rgba(0, 0, 0, 0.3); border-radius: 10px; padding: 10px; margin-top: 15px; }\n"
"        .status-item { display: flex; justify-content: space-between; margin-bottom: 5px; font-size: 0.9em; }\n"
"        .camera-controls { display: flex; gap: 10px; margin-top: 15px; }\n"
"        .camera-btn { flex: 1; padding: 8px 12px; font-size: 12px; border: none; border-radius: 8px; cursor: pointer; background: linear-gradient(45deg, #9C27B0, #7B1FA2); color: white; font-weight: bold; transition: all 0.3s ease; }\n"
"        .camera-btn:hover { transform: translateY(-1px); box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3); }\n"
"        @media (max-width: 1024px) { .main-layout { grid-template-columns: 1fr; gap: 20px; } .control-panel { order: -1; } }\n"
"        @media (max-width: 768px) { .container { padding: 10px; } h1 { font-size: 2em; } .joystick { width: 120px; height: 120px; } .joystick-knob { width: 30px; height: 30px; } .mode-buttons { grid-template-columns: 1fr; } }\n"
"    </style>\n"
"</head>\n"
"<body>\n";

static const char* html_body = 
"    <div class=\"container\">\n"
"        <h1>🚗 Elegoo Robot Car Control</h1>\n"
"        <div class=\"main-layout\">\n"
"            <div class=\"video-section\">\n"
"                <h3 style=\"margin-bottom: 15px; text-align: center;\">📹 Camera View</h3>\n"
"                <div class=\"video-container\">\n"
"                    <img id=\"video-stream\" class=\"video-stream\" src=\"/camera/stream\" alt=\"Camera Stream\" onerror=\"showVideoPlaceholder()\" onload=\"hideVideoPlaceholder()\">\n"
"                    <div id=\"video-placeholder\" class=\"video-placeholder\" style=\"display: none;\">📷 Camera not available</div>\n"
"                </div>\n"
"                <div class=\"camera-controls\">\n"
"                    <button class=\"camera-btn\" onclick=\"captureImage()\">📸 Capture</button>\n"
"                    <button class=\"camera-btn\" onclick=\"toggleFlip()\">🔄 Flip</button>\n"
"                    <button class=\"camera-btn\" onclick=\"refreshStream()\">🔄 Refresh</button>\n"
"                </div>\n"
"                <div class=\"control-section\" style=\"margin-top: 15px;\">\n"
"                    <h3>👤 Face Detection</h3>\n"
"                    <div style=\"display: flex; align-items: center; gap: 15px; margin-bottom: 10px;\">\n"
"                        <label style=\"display: flex; align-items: center; gap: 8px; cursor: pointer;\">\n"
"                            <input type=\"checkbox\" id=\"face-detection-toggle\" onchange=\"toggleFaceDetection()\" style=\"width: 18px; height: 18px;\">\n"
"                            <span>Enable Face Detection</span>\n"
"                        </label>\n"
"                        <div id=\"face-detection-status\" style=\"padding: 4px 8px; border-radius: 12px; font-size: 0.8em; background: rgba(255, 255, 255, 0.2);\">Disabled</div>\n"
"                    </div>\n"
"                    <div id=\"face-detection-info\" style=\"background: rgba(0, 0, 0, 0.3); border-radius: 8px; padding: 10px; font-size: 0.85em; display: none;\">\n"
"                        <div style=\"margin-bottom: 5px;\">Faces Detected: <span id=\"face-count\">0</span></div>\n"
"                        <div style=\"margin-bottom: 5px;\">Confidence: <span id=\"face-confidence\">--</span></div>\n"
"                        <div>Position: <span id=\"face-position\">--</span></div>\n"
"                    </div>\n"
"                </div>\n"
"            </div>\n"
"            <div class=\"control-panel\">\n"
"                <div class=\"control-section\">\n"
"                    <h3>🕹️ Movement Control</h3>\n"
"                    <div class=\"joystick-container\">\n"
"                        <div class=\"joystick\" id=\"joystick\">\n"
"                            <div class=\"joystick-knob\" id=\"joystick-knob\"></div>\n"
"                        </div>\n"
"                    </div>\n"
"                    <div class=\"speed-control\">\n"
"                        <label for=\"speed\">Speed: <span id=\"speed-value\">1000</span>ms</label>\n"
"                        <input type=\"range\" id=\"speed\" class=\"speed-slider\" min=\"500\" max=\"1500\" value=\"1000\" oninput=\"updateSpeedValue()\">\n"
"                    </div>\n"
"                </div>\n"
"                <div class=\"control-section\">\n"
"                    <h3>🤖 Chassis Modes</h3>\n"
"                    <div class=\"mode-buttons\">\n"
"                        <button class=\"mode-btn standby active\" onclick=\"setChassisMode('standby')\">⏸️ Standby</button>\n"
"                        <button class=\"mode-btn line\" onclick=\"setChassisMode('line_follow')\">📏 Line Following</button>\n"
"                        <button class=\"mode-btn obstacle\" onclick=\"setChassisMode('obstacle_avoid')\">🚧 Obstacle Avoidance</button>\n"
"                        <button class=\"mode-btn follow\" onclick=\"setChassisMode('follow')\">👤 Object Following</button>\n"
"                    </div>\n"
"                </div>\n"
"                <div class=\"control-section\">\n"
"                    <h3>⚙️ Individual Motor Control</h3>\n"
"                    <div style=\"display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-bottom: 15px;\">\n"
"                        <div style=\"background: rgba(255, 255, 255, 0.1); border-radius: 10px; padding: 10px;\">\n"
"                            <h4 style=\"margin-bottom: 10px; text-align: center;\">🔄 Left Motor</h4>\n"
"                            <div style=\"display: flex; gap: 5px; margin-bottom: 8px;\">\n"
"                                <button class=\"motor-btn\" onclick=\"controlMotor('left', 'forward')\">⬆️</button>\n"
"                                <button class=\"motor-btn\" onclick=\"controlMotor('left', 'stop')\">⏹️</button>\n"
"                                <button class=\"motor-btn\" onclick=\"controlMotor('left', 'backward')\">⬇️</button>\n"
"                            </div>\n"
"                            <input type=\"range\" id=\"left-speed\" class=\"motor-speed-slider\" min=\"0\" max=\"255\" value=\"100\" oninput=\"updateLeftSpeed()\">\n"
"                            <div style=\"text-align: center; font-size: 0.8em;\">Speed: <span id=\"left-speed-value\">100</span></div>\n"
"                        </div>\n"
"                        <div style=\"background: rgba(255, 255, 255, 0.1); border-radius: 10px; padding: 10px;\">\n"
"                            <h4 style=\"margin-bottom: 10px; text-align: center;\">🔄 Right Motor</h4>\n"
"                            <div style=\"display: flex; gap: 5px; margin-bottom: 8px;\">\n"
"                                <button class=\"motor-btn\" onclick=\"controlMotor('right', 'forward')\">⬆️</button>\n"
"                                <button class=\"motor-btn\" onclick=\"controlMotor('right', 'stop')\">⏹️</button>\n"
"                                <button class=\"motor-btn\" onclick=\"controlMotor('right', 'backward')\">⬇️</button>\n"
"                            </div>\n"
"                            <input type=\"range\" id=\"right-speed\" class=\"motor-speed-slider\" min=\"0\" max=\"255\" value=\"100\" oninput=\"updateRightSpeed()\">\n"
"                            <div style=\"text-align: center; font-size: 0.8em;\">Speed: <span id=\"right-speed-value\">100</span></div>\n"
"                        </div>\n"
"                    </div>\n"
"                    <button class=\"sync-motors-btn\" onclick=\"syncMotors()\">🔄 Sync Both Motors</button>\n"
"                </div>\n"
"                <div class=\"control-section\">\n"
"                    <h3>📊 Status</h3>\n"
"                    <div class=\"status-display\">\n"
"                        <div class=\"status-item\">\n"
"                            <span>Chassis Mode:</span>\n"
"                            <span id=\"current-mode\">Standby</span>\n"
"                        </div>\n"
"                        <div class=\"status-item\">\n"
"                            <span>Connection:</span>\n"
"                            <span id=\"connection-status\">Connected</span>\n"
"                        </div>\n"
"                        <div class=\"status-item\">\n"
"                            <span>Battery:</span>\n"
"                            <span id=\"battery-level\">--</span>\n"
"                        </div>\n"
"                    </div>\n"
"                </div>\n"
"            </div>\n"
"        </div>\n"
"    </div>\n";

static const char* html_script = 
"    <script>\n"
"        let currentMode = 'standby';\n"
"        let isMoving = false;\n"
"        let moveInterval = null;\n"
"        let joystickActive = false;\n"
"        \n"
"        function updateSpeedValue() {\n"
"            const slider = document.getElementById('speed');\n"
"            const valueSpan = document.getElementById('speed-value');\n"
"            valueSpan.textContent = slider.value + 'ms';\n"
"        }\n"
"        \n"
"        function updateLeftSpeed() {\n"
"            const slider = document.getElementById('left-speed');\n"
"            const valueSpan = document.getElementById('left-speed-value');\n"
"            valueSpan.textContent = slider.value;\n"
"        }\n"
"        \n"
"        function updateRightSpeed() {\n"
"            const slider = document.getElementById('right-speed');\n"
"            const valueSpan = document.getElementById('right-speed-value');\n"
"            valueSpan.textContent = slider.value;\n"
"        }\n"
"        \n"
"        function sendCommand(tool, params) {\n"
"            if (!params) params = {};\n"
"            const data = { tool: tool, params: params };\n"
"            fetch('/api/control', {\n"
"                method: 'POST',\n"
"                headers: { 'Content-Type': 'application/json' },\n"
"                body: JSON.stringify(data)\n"
"            }).then(response => response.json())\n"
"            .then(data => {\n"
"                console.log('Command result:', data);\n"
"                if (data.error) {\n"
"                    console.error('Error:', data.error);\n"
"                    document.getElementById('connection-status').textContent = 'Error';\n"
"                } else {\n"
"                    document.getElementById('connection-status').textContent = 'Connected';\n"
"                }\n"
"            }).catch(error => {\n"
"                console.error('Network error:', error);\n"
"                document.getElementById('connection-status').textContent = 'Disconnected';\n"
"            });\n"
"        }\n"
"        \n"
"        function setChassisMode(mode) {\n"
"            currentMode = mode;\n"
"            let displayName = '';\n"
"            let command = '';\n"
"            \n"
"            switch(mode) {\n"
"                case 'standby':\n"
"                    displayName = 'Standby';\n"
"                    command = 'self.chassis.set_mode_standby';\n"
"                    break;\n"
"                case 'line_follow':\n"
"                    displayName = 'Line Following';\n"
"                    command = 'self.chassis.set_mode_line_follow';\n"
"                    break;\n"
"                case 'obstacle_avoid':\n"
"                    displayName = 'Obstacle Avoidance';\n"
"                    command = 'self.chassis.set_mode_obstacle_avoid';\n"
"                    break;\n"
"                case 'follow':\n"
"                    displayName = 'Object Following';\n"
"                    command = 'self.chassis.set_mode_follow';\n"
"                    break;\n"
"            }\n"
"            \n"
"            document.getElementById('current-mode').textContent = displayName;\n"
"            \n"
"            // 更新按钮状态\n"
"            document.querySelectorAll('.mode-btn').forEach(btn => btn.classList.remove('active'));\n"
"            event.target.classList.add('active');\n"
"            \n"
"            // 发送模式切换命令\n"
"            if (command) sendCommand(command);\n"
"        }\n"
"        \n"
"        function controlMotor(motor, direction) {\n"
"            let speed = 100;\n"
"            let command = '';\n"
"            \n"
"            if (motor === 'left') {\n"
"                speed = parseInt(document.getElementById('left-speed').value);\n"
"                command = 'self.chassis.control_left_motor';\n"
"            } else if (motor === 'right') {\n"
"                speed = parseInt(document.getElementById('right-speed').value);\n"
"                command = 'self.chassis.control_right_motor';\n"
"            }\n"
"            \n"
"            const params = {\n"
"                direction: direction,\n"
"                speed: speed\n"
"            };\n"
"            \n"
"            sendCommand(command, params);\n"
"        }\n"
"        \n"
"        function syncMotors() {\n"
"            const leftDirection = 'forward';\n"
"            const leftSpeed = parseInt(document.getElementById('left-speed').value);\n"
"            const rightDirection = 'forward';\n"
"            const rightSpeed = parseInt(document.getElementById('right-speed').value);\n"
"            \n"
"            const params = {\n"
"                left_direction: leftDirection,\n"
"                left_speed: leftSpeed,\n"
"                right_direction: rightDirection,\n"
"                right_speed: rightSpeed\n"
"            };\n"
"            \n"
"            sendCommand('self.chassis.control_both_motors', params);\n"
"        }\n"
"        \n"
"        function moveRobot(direction) {\n"
"            if (currentMode !== 'standby') return; // 只在待机模式下允许手动控制\n"
"            \n"
"            const speed = parseInt(document.getElementById('speed').value);\n"
"            let command = '';\n"
"            let params = { steps: 1, speed: speed };\n"
"            \n"
"            switch(direction) {\n"
"                case 'forward':\n"
"                    command = 'self.chassis.go_forward';\n"
"                    break;\n"
"                case 'backward':\n"
"                    command = 'self.chassis.go_back';\n"
"                    break;\n"
"                case 'left':\n"
"                    command = 'self.chassis.turn_left';\n"
"                    break;\n"
"                case 'right':\n"
"                    command = 'self.chassis.turn_right';\n"
"                    break;\n"
"            }\n"
"            \n"
"            if (command) sendCommand(command, params);\n"
"        }\n"
"        \n"
"        function stopRobot() {\n"
"            sendCommand('self.chassis.stop');\n"
"        }\n"
"        \n"
"        // 摇杆控制\n"
"        function initJoystick() {\n"
"            const joystick = document.getElementById('joystick');\n"
"            const knob = document.getElementById('joystick-knob');\n"
"            const rect = joystick.getBoundingClientRect();\n"
"            const centerX = rect.width / 2;\n"
"            const centerY = rect.height / 2;\n"
"            const maxDistance = centerX - 20;\n"
"            \n"
"            function handleMove(clientX, clientY) {\n"
"                const rect = joystick.getBoundingClientRect();\n"
"                const x = clientX - rect.left - centerX;\n"
"                const y = clientY - rect.top - centerY;\n"
"                const distance = Math.sqrt(x * x + y * y);\n"
"                \n"
"                if (distance <= maxDistance) {\n"
"                    knob.style.left = (centerX + x) + 'px';\n"
"                    knob.style.top = (centerY + y) + 'px';\n"
"                } else {\n"
"                    const angle = Math.atan2(y, x);\n"
"                    const limitedX = Math.cos(angle) * maxDistance;\n"
"                    const limitedY = Math.sin(angle) * maxDistance;\n"
"                    knob.style.left = (centerX + limitedX) + 'px';\n"
"                    knob.style.top = (centerY + limitedY) + 'px';\n"
"                }\n"
"                \n"
"                // 确定方向并发送命令\n"
"                const normalizedX = x / maxDistance;\n"
"                const normalizedY = y / maxDistance;\n"
"                \n"
"                if (Math.abs(normalizedX) > 0.3 || Math.abs(normalizedY) > 0.3) {\n"
"                    if (Math.abs(normalizedY) > Math.abs(normalizedX)) {\n"
"                        if (normalizedY < -0.3) moveRobot('forward');\n"
"                        else if (normalizedY > 0.3) moveRobot('backward');\n"
"                    } else {\n"
"                        if (normalizedX < -0.3) moveRobot('left');\n"
"                        else if (normalizedX > 0.3) moveRobot('right');\n"
"                    }\n"
"                }\n"
"            }\n"
"            \n"
"            function resetKnob() {\n"
"                knob.style.left = '50%';\n"
"                knob.style.top = '50%';\n"
"                joystickActive = false;\n"
"                stopRobot();\n"
"            }\n"
"            \n"
"            // 鼠标事件\n"
"            joystick.addEventListener('mousedown', (e) => {\n"
"                joystickActive = true;\n"
"                handleMove(e.clientX, e.clientY);\n"
"            });\n"
"            \n"
"            document.addEventListener('mousemove', (e) => {\n"
"                if (joystickActive) handleMove(e.clientX, e.clientY);\n"
"            });\n"
"            \n"
"            document.addEventListener('mouseup', resetKnob);\n"
"            \n"
"            // 触摸事件\n"
"            joystick.addEventListener('touchstart', (e) => {\n"
"                e.preventDefault();\n"
"                joystickActive = true;\n"
"                const touch = e.touches[0];\n"
"                handleMove(touch.clientX, touch.clientY);\n"
"            });\n"
"            \n"
"            document.addEventListener('touchmove', (e) => {\n"
"                if (joystickActive) {\n"
"                    e.preventDefault();\n"
"                    const touch = e.touches[0];\n"
"                    handleMove(touch.clientX, touch.clientY);\n"
"                }\n"
"            });\n"
"            \n"
"            document.addEventListener('touchend', (e) => {\n"
"                e.preventDefault();\n"
"                resetKnob();\n"
"            });\n"
"        }\n"
"        \n"
"        // 摄像头控制\n"
"        function captureImage() {\n"
"            window.open('/camera/capture', '_blank');\n"
"        }\n"
"        \n"
"        function toggleFlip() {\n"
"            fetch('/camera/flip', { method: 'POST' })\n"
"            .then(response => response.json())\n"
"            .then(data => console.log('Camera flip:', data))\n"
"            .catch(error => console.error('Camera flip error:', error));\n"
"        }\n"
"        \n"
"        function refreshStream() {\n"
"            const img = document.getElementById('video-stream');\n"
"            img.src = '/camera/stream?' + new Date().getTime();\n"
"        }\n"
"        \n"
"        // 人脸检测控制函数\n"
"        function toggleFaceDetection() {\n"
"            const checkbox = document.getElementById('face-detection-toggle');\n"
"            const status = document.getElementById('face-detection-status');\n"
"            const info = document.getElementById('face-detection-info');\n"
"            \n"
"            const enabled = checkbox.checked;\n"
"            \n"
"            // 发送人脸检测开关请求\n"
"            fetch('/api/face_detection/toggle', {\n"
"                method: 'POST',\n"
"                headers: { 'Content-Type': 'application/json' },\n"
"                body: JSON.stringify({ enabled: enabled })\n"
"            })\n"
"            .then(response => response.json())\n"
"            .then(data => {\n"
"                if (data.success) {\n"
"                    if (enabled) {\n"
"                        status.textContent = 'Enabled';\n"
"                        status.style.background = 'rgba(76, 175, 80, 0.8)';\n"
"                        info.style.display = 'block';\n"
"                        // 开始定期获取检测结果\n"
"                        startFaceDetectionPolling();\n"
"                    } else {\n"
"                        status.textContent = 'Disabled';\n"
"                        status.style.background = 'rgba(255, 255, 255, 0.2)';\n"
"                        info.style.display = 'none';\n"
"                        // 停止获取检测结果\n"
"                        stopFaceDetectionPolling();\n"
"                    }\n"
"                    // 刷新视频流以应用新设置\n"
"                    refreshStream();\n"
"                } else {\n"
"                    console.error('Failed to toggle face detection');\n"
"                    // 恢复复选框状态\n"
"                    checkbox.checked = !enabled;\n"
"                }\n"
"            })\n"
"            .catch(error => {\n"
"                console.error('Error toggling face detection:', error);\n"
"                // 恢复复选框状态\n"
"                checkbox.checked = !enabled;\n"
"            });\n"
"        }\n"
"        \n"
"        let faceDetectionInterval = null;\n"
"        \n"
"        function startFaceDetectionPolling() {\n"
"            if (faceDetectionInterval) return;\n"
"            \n"
"            faceDetectionInterval = setInterval(() => {\n"
"                fetch('/api/face_detection/result')\n"
"                .then(response => response.json())\n"
"                .then(data => {\n"
"                    if (data.success) {\n"
"                        document.getElementById('face-count').textContent = data.face_count || 0;\n"
"                        document.getElementById('face-confidence').textContent = \n"
"                            data.confidence ? (data.confidence * 100).toFixed(1) + '%' : '--';\n"
"                        \n"
"                        if (data.box && data.face_count > 0) {\n"
"                            document.getElementById('face-position').textContent = \n"
"                                `(${data.box.x}, ${data.box.y}) ${data.box.w}×${data.box.h}`;\n"
"                        } else {\n"
"                            document.getElementById('face-position').textContent = '--';\n"
"                        }\n"
"                    }\n"
"                })\n"
"                .catch(error => {\n"
"                    console.error('Error getting face detection result:', error);\n"
"                });\n"
"            }, 1000); // 每秒更新一次\n"
"        }\n"
"        \n"
"        function stopFaceDetectionPolling() {\n"
"            if (faceDetectionInterval) {\n"
"                clearInterval(faceDetectionInterval);\n"
"                faceDetectionInterval = null;\n"
"            }\n"
"        }\n"
"        \n"
"        function showVideoPlaceholder() {\n"
"            document.getElementById('video-stream').style.display = 'none';\n"
"            document.getElementById('video-placeholder').style.display = 'block';\n"
"        }\n"
"        \n"
"        function hideVideoPlaceholder() {\n"
"            document.getElementById('video-stream').style.display = 'block';\n"
"            document.getElementById('video-placeholder').style.display = 'none';\n"
"        }\n"
"        \n"
"        // 获取电池状态\n"
"        function updateBatteryStatus() {\n"
"            sendCommand('self.battery.get_level');\n"
"        }\n"
"        \n"
"        // 页面加载完成后初始化\n"
"        document.addEventListener('DOMContentLoaded', function() {\n"
"            initJoystick();\n"
"            updateBatteryStatus();\n"
"            setInterval(updateBatteryStatus, 30000); // 每30秒更新一次电池状态\n"
"            \n"
"            // 初始化人脸检测状态\n"
"            const faceDetectionInfo = document.getElementById('face-detection-info');\n"
"            if (faceDetectionInfo) {\n"
"                faceDetectionInfo.style.display = 'none'; // 默认隐藏检测信息\n"
"            }\n"
"        });\n"
"    </script>\n"
"</body>\n"
"</html>\n";

// 摄像头流处理函数
static esp_err_t camera_stream_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char *part_buf[64];
    
    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
    static const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
    
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    while (true) {
        CameraManager* camera = GetCamera();
        if (!camera) {
            ESP_LOGE(TAG, "摄像头未初始化");
            res = ESP_FAIL;
            break;
        }
        
        // 捕获图像
        camera_fb_t* fb = camera->GetFrame();
        if (!fb) {
            ESP_LOGE(TAG, "摄像头捕获失败");
            res = ESP_FAIL;
        } else {
            res = ESP_OK;
        }
        
        if (res == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        
        if (fb) {
            //esp_camera_fb_return(fb);
            camera->ReturnFrame(fb);
        }
        
        if (res != ESP_OK) {
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(30)); // 增加延迟以减少CPU负载
    }
    
    return res;
}

// 摄像头单张图片捕获
static esp_err_t camera_capture_handler(httpd_req_t *req) {
    CameraManager* camera = GetCamera();
    if (!camera) {
        ESP_LOGE(TAG, "摄像头未初始化");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // 捕获图像
    camera_fb_t* fb = camera->GetFrame();
    if (!fb) {
        ESP_LOGE(TAG, "摄像头捕获失败");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    camera->ReturnFrame(fb);
    
    return res;
}

// 人脸检测开关处理函数（简化版本）
static esp_err_t face_detection_toggle_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    if (remaining >= sizeof(buf)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *enabled = cJSON_GetObjectItem(json, "enabled");
    if (cJSON_IsBool(enabled)) {
        bool new_state = cJSON_IsTrue(enabled);
        face_detection_enabled = new_state;
        
        ESP_LOGI(TAG, "人脸检测状态: %s (注意：当前版本不支持人脸检测)", new_state ? "启用" : "禁用");
    }
    
    cJSON_Delete(json);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"status\":\"ok\",\"note\":\"Face detection not supported in current version\"}", HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

// 人脸检测结果处理函数（简化版本）
static esp_err_t face_detection_result_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // 返回空的人脸检测结果，因为当前版本不支持人脸检测
    cJSON *json = cJSON_CreateObject();
    cJSON *faces_array = cJSON_CreateArray();
    
    cJSON_AddNumberToObject(json, "face_count", 0);
    cJSON_AddBoolToObject(json, "detection_enabled", false);
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time() / 1000);
    cJSON_AddStringToObject(json, "note", "Face detection not supported in current version");
    cJSON_AddItemToObject(json, "faces", faces_array);
    
    char *json_string = cJSON_Print(json);
    httpd_resp_send(req, json_string, HTTPD_RESP_USE_STRLEN);
    
    free(json_string);
    cJSON_Delete(json);
    
    return ESP_OK;
}

// 摄像头翻转处理函数
static esp_err_t camera_flip_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    CameraManager* camera = GetCamera();
    if (!camera) {
        httpd_resp_send(req, "{\"error\":\"Camera not initialized\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    // 切换垂直翻转状态（简化实现，假设默认为false）
    static bool current_flip = false;
    current_flip = !current_flip;
    camera->SetVFlip(current_flip);
    
    ESP_LOGI(TAG, "摄像头垂直翻转: %s", current_flip ? "启用" : "禁用");
    
    char response[128];
    snprintf(response, sizeof(response), 
             "{\"status\":\"ok\",\"vertical_flip\":%s}", 
             current_flip ? "true" : "false");
    
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

// HTTP处理函数
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // 发送HTML头部
    httpd_resp_send_chunk(req, html_header, HTTPD_RESP_USE_STRLEN);
    
    // 发送HTML主体
    httpd_resp_send_chunk(req, html_body, HTTPD_RESP_USE_STRLEN);
    
    // 发送JavaScript脚本
    httpd_resp_send_chunk(req, html_script, HTTPD_RESP_USE_STRLEN);
    
    // 结束响应
    httpd_resp_send_chunk(req, nullptr, 0);
    
    return ESP_OK;
}

static esp_err_t control_handler(httpd_req_t *req) {
    char content[WEB_SERVER_CONTENT_BUFFER_SIZE];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    ESP_LOGI(TAG, "收到控制命令: %s", content);
    
    // 解析JSON
    cJSON *json = cJSON_Parse(content);
    if (json == nullptr) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, "{\"error\":\"Invalid JSON\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    cJSON *tool_item = cJSON_GetObjectItem(json, "tool");
    cJSON *params_item = cJSON_GetObjectItem(json, "params");
    
    if (!cJSON_IsString(tool_item)) {
        cJSON_Delete(json);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, "{\"error\":\"Missing tool parameter\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    std::string tool_name = tool_item->valuestring;
    
    // 构建MCP消息
    cJSON *mcp_message = cJSON_CreateObject();
    cJSON_AddStringToObject(mcp_message, "jsonrpc", "2.0");
    cJSON_AddStringToObject(mcp_message, "method", "tools/call");
    cJSON_AddNumberToObject(mcp_message, "id", 1);
    
    cJSON *mcp_params = cJSON_CreateObject();
    cJSON_AddStringToObject(mcp_params, "name", tool_name.c_str());
    
    if (cJSON_IsObject(params_item)) {
        cJSON *arguments = cJSON_CreateObject();
        cJSON *param = params_item->child;
        while (param != nullptr) {
            cJSON *param_copy = cJSON_Duplicate(param, 1);
            cJSON_AddItemToObject(arguments, param->string, param_copy);
            param = param->next;
        }
        cJSON_AddItemToObject(mcp_params, "arguments", arguments);
    } else {
        cJSON_AddObjectToObject(mcp_params, "arguments");
    }
    
    cJSON_AddItemToObject(mcp_message, "params", mcp_params);
    
    // 发送到MCP服务器
    char *mcp_string = cJSON_Print(mcp_message);
    ESP_LOGI(TAG, "发送MCP消息: %s", mcp_string);
    
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.ParseMessage(mcp_string);
    
    free(mcp_string);
    cJSON_Delete(mcp_message);
    cJSON_Delete(json);
    
    // 返回成功响应
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"success\":true}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t elegoo_web_server_init(void) {
    // 预防性清理：确保没有残留的服务器实例
    if (server != nullptr) {
        ESP_LOGW(TAG, "发现残留的Web服务器实例，正在清理...");
        httpd_stop(server);
        server = nullptr;
        vTaskDelay(500 / portTICK_PERIOD_MS); // 等待500ms确保端口释放
    }
    
    // 检查内存状态
    size_t free_heap = esp_get_free_heap_size();
    size_t min_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "启动前内存状态: 当前可用=%u字节, 历史最小=%u字节", 
             (unsigned int)free_heap, (unsigned int)min_heap);
    
    // 如果可用内存不足，尝试释放内存
    if (free_heap < MIN_FREE_HEAP_SIZE) {
        ESP_LOGW(TAG, "可用内存不足(%u < %u)，尝试释放内存...", 
                 (unsigned int)free_heap, (unsigned int)MIN_FREE_HEAP_SIZE);
        
        // 强制垃圾回收（如果可用）
        #ifdef CONFIG_HEAP_TRACING
        heap_caps_dump_all();
        #endif
        
        // 再次检查内存
        free_heap = esp_get_free_heap_size();
        ESP_LOGI(TAG, "内存清理后: 当前可用=%u字节", (unsigned int)free_heap);
    }
    
    // 检查摄像头是否可用
    CameraManager* camera = GetCamera();
    bool camera_available = (camera != nullptr);
    
    if (camera_available) {
        ESP_LOGI(TAG, "摄像头已初始化，Web服务器将支持摄像头功能");
        face_detection_enabled = false; // 当前版本不支持人脸检测
    } else {
        ESP_LOGW(TAG, "摄像头未初始化，Web服务器将在无摄像头模式下运行");
        face_detection_enabled = false;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = ELEGOO_WEB_SERVER_PORT;
    config.max_uri_handlers = 8;  // 减少URI处理器数量
    config.max_resp_headers = 6;  // 减少响应头数量
    config.stack_size = WEB_SERVER_STACK_SIZE;    // 使用配置文件中的栈大小
    config.task_priority = 4;     // 降低任务优先级，避免与其他任务冲突
    config.core_id = tskNO_AFFINITY;  // 允许任务在任意核心运行
    config.max_open_sockets = 4;  // 限制最大连接数
    config.backlog_conn = 2;      // 减少待处理连接数
    config.lru_purge_enable = true;  // 启用LRU清理，释放不活跃连接
    //config.so_reuseport = true;   // 启用端口重用，解决端口占用问题
    
    ESP_LOGI(TAG, "启动Web服务器，端口: %d", config.server_port);
    
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动HTTP服务器失败: %s", esp_err_to_name(ret));
        
        // 根据错误类型提供具体的解决建议
        switch (ret) {
            case ESP_ERR_HTTPD_TASK:
                ESP_LOGE(TAG, "HTTP服务器任务创建失败 - 可能原因:");
                ESP_LOGE(TAG, "1. 内存不足 (当前可用: %u bytes)", (unsigned int)esp_get_free_heap_size());
                ESP_LOGE(TAG, "2. 任务数量达到上限");
                ESP_LOGE(TAG, "3. 栈大小不足");
                break;
            case ESP_ERR_HTTPD_ALLOC_MEM:
                ESP_LOGE(TAG, "HTTP服务器内存分配失败");
                break;
            case ESP_ERR_HTTPD_INVALID_REQ:
                ESP_LOGE(TAG, "HTTP服务器配置无效");
                break;
            case ESP_FAIL:
                ESP_LOGE(TAG, "HTTP服务器启动失败 - 可能是端口被占用");
                ESP_LOGI(TAG, "尝试强制停止可能存在的服务器实例...");
                // 尝试停止可能存在的服务器实例
                if (server != nullptr) {
                    httpd_stop(server);
                    server = nullptr;
                }
                break;
            default:
                ESP_LOGE(TAG, "未知错误: 0x%x", ret);
                break;
        }
        
        // 尝试延迟后重试一次
        ESP_LOGW(TAG, "等待%dms后重试启动HTTP服务器...", WEB_SERVER_INIT_RETRY_DELAY_MS);
        vTaskDelay(WEB_SERVER_INIT_RETRY_DELAY_MS / portTICK_PERIOD_MS);
        
        // 再次检查内存状态
        ESP_LOGI(TAG, "重试前内存状态: %u bytes", (unsigned int)esp_get_free_heap_size());
        
        ret = httpd_start(&server, &config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "重试启动HTTP服务器仍然失败: %s", esp_err_to_name(ret));
            
            // 如果仍然失败，尝试使用不同的端口
            if (ret == ESP_FAIL) {
                ESP_LOGW(TAG, "尝试使用备用端口8080...");
                config.server_port = 8080;
                ret = httpd_start(&server, &config);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "使用备用端口8080启动HTTP服务器成功");
                } else {
                    ESP_LOGE(TAG, "使用备用端口8080仍然失败: %s", esp_err_to_name(ret));
                    return ret;
                }
            } else {
                return ret;
            }
        } else {
            ESP_LOGI(TAG, "重试启动HTTP服务器成功");
        }
    }
    
    // 注册基本URI处理器
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &root_uri);
    
    httpd_uri_t control_uri = {
        .uri = "/api/control",
        .method = HTTP_POST,
        .handler = control_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &control_uri);
    
    // 只在摄像头可用时注册摄像头相关的端点
    if (camera_available) {
        ESP_LOGI(TAG, "注册摄像头相关API端点");
        
        httpd_uri_t camera_stream_uri = {
            .uri = "/camera/stream",
            .method = HTTP_GET,
            .handler = camera_stream_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server, &camera_stream_uri);
        
        httpd_uri_t camera_capture_uri = {
            .uri = "/camera/capture",
            .method = HTTP_GET,
            .handler = camera_capture_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server, &camera_capture_uri);
        
        httpd_uri_t camera_flip_uri = {
            .uri = "/camera/flip",
            .method = HTTP_POST,
            .handler = camera_flip_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server, &camera_flip_uri);
        
        httpd_uri_t face_detection_toggle_uri = {
            .uri = "/api/face_detection/toggle",
            .method = HTTP_POST,
            .handler = face_detection_toggle_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server, &face_detection_toggle_uri);
        
        httpd_uri_t face_detection_result_uri = {
            .uri = "/api/face_detection/result",
            .method = HTTP_GET,
            .handler = face_detection_result_handler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server, &face_detection_result_uri);
    } else {
        ESP_LOGW(TAG, "摄像头不可用，跳过摄像头相关API端点注册");
    }
    
    // 获取并打印IP地址
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "Elegoo机器人小车Web控制界面: http://" IPSTR ":%d", 
                 IP2STR(&ip_info.ip), config.server_port);
    }
    
    ESP_LOGI(TAG, "Elegoo Web控制服务器启动成功");
    return ESP_OK;
}

esp_err_t elegoo_web_server_stop(void) {
    if (server == nullptr) {
        ESP_LOGW(TAG, "Web服务器未运行");
        return ESP_OK;
    }
    
    esp_err_t ret = httpd_stop(server);
    if (ret == ESP_OK) {
        server = nullptr;
        ESP_LOGI(TAG, "Web服务器已停止");
    } else {
        ESP_LOGE(TAG, "停止Web服务器失败: %s", esp_err_to_name(ret));
    }
    
    // 摄像头由Board管理，这里不需要清理
    face_detection_enabled = false;
    
    return ret;
}