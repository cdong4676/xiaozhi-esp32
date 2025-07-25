#ifndef _ELEGOO_WEB_SERVER_H_
#define _ELEGOO_WEB_SERVER_H_

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化Elegoo机器人小车Web控制服务器
 * 
 * @return esp_err_t ESP_OK表示成功，其他值表示失败
 */
esp_err_t elegoo_web_server_init(void);

/**
 * @brief 停止Elegoo机器人小车Web控制服务器
 * 
 * @return esp_err_t ESP_OK表示成功，其他值表示失败
 */
esp_err_t elegoo_web_server_stop(void);

#ifdef __cplusplus
}
#endif

#define ELEGOO_WEB_SERVER_PORT 80

#endif  // _ELEGOO_WEB_SERVER_H_