#ifndef _OTTO_WEB_SERVER_H_
#define _OTTO_WEB_SERVER_H_

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化Otto机器人Web控制服务器
 * 
 * @return esp_err_t ESP_OK表示成功，其他值表示失败
 */
esp_err_t otto_web_server_init(void);

/**
 * @brief 停止Otto机器人Web控制服务器
 * 
 * @return esp_err_t ESP_OK表示成功，其他值表示失败
 */
esp_err_t otto_web_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif // _OTTO_WEB_SERVER_H_