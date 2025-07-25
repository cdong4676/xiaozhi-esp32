#include "otto_web_server.h"
#include "config.h"
#include "mcp_server.h"
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <cJSON.h>
#include <string>

// 定义MIN宏（如果未定义）
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define TAG "OttoWebServer"

static httpd_handle_t server = nullptr;

// HTML页面内容 - 分离为多个字符串以避免编译问题
static const char* html_header = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"    <meta charset=\"UTF-8\">\n"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, orientation=landscape\">\n"
"    <title>Otto Robot Control</title>\n"
"    <style>\n"
"        body { font-family: Arial, sans-serif; margin: 0; padding: 10px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; min-height: 100vh; overflow-x: auto; }\n"
"        .container { max-width: 100%; margin: 0 auto; background: rgba(255, 255, 255, 0.1); border-radius: 15px; padding: 15px; backdrop-filter: blur(10px); box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3); }\n"
"        h1 { text-align: center; margin-bottom: 15px; font-size: 1.5em; text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.3); }\n"
"        .main-content { display: grid; grid-template-columns: 1fr; gap: 20px; justify-content: center; max-width: 800px; margin: 0 auto; }\n"
"        .control-panel { min-width: 300px; width: 100%; }\n"

"        .control-section { margin-bottom: 15px; padding: 12px; background: rgba(255, 255, 255, 0.1); border-radius: 10px; }\n"
"        .control-section h3 { margin-top: 0; margin-bottom: 10px; color: #fff; border-bottom: 2px solid rgba(255, 255, 255, 0.3); padding-bottom: 5px; font-size: 1em; }\n"
"        .button-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 15px; }\n"
"        .control-btn { padding: clamp(8px, 1.5vw, 16px) clamp(12px, 2vw, 24px); font-size: clamp(12px, 1.4vw, 16px); border: none; border-radius: 8px; cursor: pointer; transition: all 0.3s ease; background: linear-gradient(45deg, #4CAF50, #45a049); color: white; font-weight: bold; text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.3); }\n"
"        .control-btn:hover { transform: translateY(-2px); box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3); }\n"
"        .control-btn:active { transform: translateY(0); }\n"
"        .control-btn.danger { background: linear-gradient(45deg, #f44336, #d32f2f); }\n"
"        .control-btn.warning { background: linear-gradient(45deg, #ff9800, #f57c00); }\n"
"        .control-btn.info { background: linear-gradient(45deg, #2196F3, #1976D2); }\n"
"        .control-btn.attack { background: linear-gradient(45deg, #9C27B0, #7B1FA2); }\n"
"        .control-layout { display: flex; flex-wrap: wrap; gap: 3vw; align-items: flex-start; justify-content: center; width: 100%; }\n"
"        .direction-pad { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: 1fr 1fr 1fr; gap: 1vw; width: 100%; max-width: 300px; }\n"
"        .attack-buttons { display: flex; gap: 1.5vw; flex-wrap: wrap; justify-content: center; }\n"
"        .attack-btn { width: 5vw; height: 5vw; min-width: 60px; min-height: 60px; max-width: 100px; max-height: 100px; border: none; border-radius: 50%; cursor: pointer; background: linear-gradient(45deg, #9C27B0, #7B1FA2); color: white; font-weight: bold; font-size: clamp(12px, 1.4vw, 16px); display: flex; align-items: center; justify-content: center; }\n"
"        .attack-btn:hover { transform: translateY(-1px); box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3); }\n"
"        .direction-btn { width: 4vw; height: 4vw; min-width: 50px; min-height: 50px; max-width: 80px; max-height: 80px; border: none; border-radius: 8px; cursor: pointer; background: linear-gradient(45deg, #2196F3, #1976D2); color: white; font-weight: bold; font-size: clamp(12px, 1.5vw, 18px); display: flex; align-items: center; justify-content: center; }\n"
"        .direction-btn:hover { background: linear-gradient(45deg, #1976D2, #1565C0); }\n"
"        .direction-btn.danger { background: linear-gradient(45deg, #ff9800, #f57c00); }\n"
"        .direction-btn.danger:hover { background: linear-gradient(45deg, #f57c00, #ef6c00); }\n"
"        .direction-btn:nth-child(1) { grid-column: 2; grid-row: 1; }\n"
"        .direction-btn:nth-child(2) { grid-column: 1; grid-row: 2; }\n"
"        .direction-btn:nth-child(3) { grid-column: 2; grid-row: 2; }\n"
"        .direction-btn:nth-child(4) { grid-column: 3; grid-row: 2; }\n"
"        .direction-btn:nth-child(5) { grid-column: 2; grid-row: 3; }\n"
"@media (min-width: 600px) { .control-layout { justify-content: space-between; align-items: flex-start; } .direction-pad { flex: 0 0 auto; } .attack-buttons { flex: 0 0 auto; gap: 2.5vw; margin-left: auto; align-self: center; } }\n"
"        @media (max-width: 599px) { .control-layout { flex-direction: column; align-items: center; gap: 20px; } .attack-buttons { order: 2; } }\n"
"        .slider-container { margin: 8px 0; }\n"
"        .slider-container label { display: block; margin-bottom: 3px; font-weight: bold; font-size: 0.8em; }\n"
"        .slider { width: 100%; height: 6px; border-radius: 5px; background: rgba(255, 255, 255, 0.3); outline: none; -webkit-appearance: none; }\n"
"        .slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 16px; height: 16px; border-radius: 50%; background: #4CAF50; cursor: pointer; }\n"
"        .slider::-moz-range-thumb { width: 16px; height: 16px; border-radius: 50%; background: #4CAF50; cursor: pointer; border: none; }\n"

"        @media (max-width: 768px) { .main-content { max-width: 400px; } .container { padding: 10px; } h1 { font-size: 1.3em; } .control-panel { min-width: auto; } .direction-pad { max-width: 200px; } .direction-btn { width: 12vw; height: 12vw; min-width: 45px; min-height: 45px; max-width: 60px; max-height: 60px; } .attack-btn { width: 15vw; height: 15vw; min-width: 50px; min-height: 50px; max-width: 70px; max-height: 70px; } }\n"
"    </style>\n"
"</head>\n"
"<body>\n";

static const char* html_body = 
"    <div class=\"container\">\n"
"        <h1>Otto Robot Control</h1>\n"
"        <div class=\"main-content\">\n"
"            <div class=\"control-panel\">\n"
"                <div class=\"control-section\">\n"
"                    <h3>Control</h3>\n"
"                    <div class=\"control-layout\">\n"
"                        <div class=\"direction-pad\">\n"
"                            <button class=\"direction-btn\" onclick=\"walk(1)\">↑</button>\n"
"                            <button class=\"direction-btn\" onclick=\"turn(3)\">←</button>\n"
"                            <button class=\"direction-btn danger\" onclick=\"stop()\">STOP</button>\n"
"                            <button class=\"direction-btn\" onclick=\"turn(4)\">→</button>\n"
"                            <button class=\"direction-btn\" onclick=\"walk(2)\">↓</button>\n"
"                        </div>\n"
"                        <div class=\"attack-buttons\">\n"
"                            <button class=\"attack-btn\" onclick=\"attack(3)\">Left hand</button>\n"
"                            <button class=\"attack-btn\" onclick=\"attack(4)\">Right hand</button>\n"
"                            <button class=\"attack-btn\" onclick=\"attack(5)\">Both hands</button>\n"
"                        </div>\n"
"                    </div>\n"
"                    <div class=\"button-grid\" style=\"margin-top: 20px;\">\n"
"                        <button class=\"control-btn info\" onclick=\"wheelMode()\">Wheel Mode</button>\n"
"                        <button class=\"control-btn info\" onclick=\"footMode()\">Foot Mode</button>\n"
"                    </div>\n"
"                    <div class=\"slider-container\" style=\"margin-top: 20px;\">\n"
"                        <label for=\"steps\">Steps: <span id=\"steps-value\">3</span></label>\n"
"                        <input type=\"range\" id=\"steps\" class=\"slider\" min=\"1\" max=\"10\" value=\"3\" oninput=\"updateSliderValue('steps', 'steps-value')\">\n"
"                    </div>\n"
"                    <div class=\"slider-container\">\n"
"                        <label for=\"speed\">Speed: <span id=\"speed-value\">1000</span>ms</label>\n"
"                        <input type=\"range\" id=\"speed\" class=\"slider\" min=\"500\" max=\"1500\" value=\"1000\" oninput=\"updateSliderValue('speed', 'speed-value')\">\n"
"                    </div>\n"
"                </div>\n"
"            </div>\n"
"        </div>\n"
"    </div>\n";

static const char* html_script = 
"    <script>\n"
"        function updateSliderValue(sliderId, valueId) {\n"
"            var slider = document.getElementById(sliderId);\n"
"            var valueSpan = document.getElementById(valueId);\n"
"            valueSpan.textContent = slider.value + (sliderId === 'speed' ? 'ms' : '');\n"
"        }\n"

"        function sendCommand(tool, params) {\n"
"            if (!params) params = {};\n"
"            var data = { tool: tool, params: params };\n"
"            fetch('/api/control', {\n"
"                method: 'POST',\n"
"                headers: { 'Content-Type': 'application/json' },\n"
"                body: JSON.stringify(data)\n"
"            }).then(function(response) { return response.json(); })\n"
"            .then(function(data) {\n"
"                console.log('Command result:', data);\n"
"                if (data.error) alert('Error: ' + data.error);\n"
"            }).catch(function(error) {\n"
"                console.error('Error:', error);\n"
"                alert('Network error: ' + error.message);\n"
"            });\n"
"        }\n"
"        function walk(direction) {\n"
"            var steps = parseInt(document.getElementById('steps').value);\n"
"            var speed = parseInt(document.getElementById('speed').value);\n"
"            sendCommand('self.otto.walk_forward', { steps: steps, speed: speed, direction: direction });\n"
"        }\n"
"        function turn(direction) {\n"
"            var steps = parseInt(document.getElementById('steps').value);\n"
"            var speed = parseInt(document.getElementById('speed').value);\n"
"            sendCommand('self.otto.turn_left', { steps: steps, speed: speed, direction: direction, arm_swing: 50 });\n"
"        }\n"
"        function wheelMode() { sendCommand('self.otto.wheel_mode'); }\n"
"        function footMode() { sendCommand('self.otto.foot_mode'); }\n"
"        function armWave() {\n"
"            var steps = parseInt(document.getElementById('steps').value);\n"
"            var speed = parseInt(document.getElementById('speed').value);\n"
"            sendCommand('self.otto.arm_wave', { steps: steps, speed: speed });\n"
"        }\n"
"        function stop() { sendCommand('self.otto.stop'); }\n"
"        function attack(direction) {\n"
"            var cycles = parseInt(document.getElementById('steps').value);\n"
"            var speed = parseInt(document.getElementById('speed').value);\n"
"            sendCommand('self.otto.attack', { cycles: cycles, speed: speed, direction: direction });\n"
"        }\n"



"    </script>\n"
"</body>\n"
"</html>\n";

// HTTP处理函数
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    
    // 发送HTML头部
    httpd_resp_send_chunk(req, html_header, HTTPD_RESP_USE_STRLEN);
    
    // 发送HTML主体
    httpd_resp_send_chunk(req, html_body, HTTPD_RESP_USE_STRLEN);
    
    // 发送JavaScript脚本
    httpd_resp_send_chunk(req, html_script, HTTPD_RESP_USE_STRLEN);
    
    // 结束响应
    httpd_resp_send_chunk(req, NULL, 0);
    
    return ESP_OK;
}

static esp_err_t control_handler(httpd_req_t *req) {
    char content[1024];
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
        httpd_resp_send(req, "{\"error\":\"Invalid JSON\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    cJSON *tool_item = cJSON_GetObjectItem(json, "tool");
    cJSON *params_item = cJSON_GetObjectItem(json, "params");
    
    if (!cJSON_IsString(tool_item)) {
        cJSON_Delete(json);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
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
    httpd_resp_send(req, "{\"success\":true}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}



esp_err_t otto_web_server_init(void) {
    if (server != nullptr) {
        ESP_LOGW(TAG, "Web服务器已经在运行");
        return ESP_OK;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = OTTO_WEB_SERVER_PORT;
    config.max_uri_handlers = 8;
    config.max_resp_headers = 8;
    config.stack_size = 8192;
    
    ESP_LOGI(TAG, "启动Web服务器，端口: %d", config.server_port);
    
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动HTTP服务器失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 注册URI处理器
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
    

    
    // 获取并打印IP地址
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "Web控制界面地址: http://" IPSTR ":%d", 
                 IP2STR(&ip_info.ip), config.server_port);
    }
    
    ESP_LOGI(TAG, "Otto Web控制服务器启动成功");
    return ESP_OK;
}

esp_err_t otto_web_server_stop(void) {
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
    
    return ret;
}