#include "otto_emoji_display.h"

#include <esp_log.h>

#include "display/lvgl_display/lvgl_theme.h"

namespace {
constexpr char TAG[] = "ElegooOttoEmoji";
}

OttoEmojiDisplay::OttoEmojiDisplay(esp_lcd_panel_io_handle_t panel_io,
                                   esp_lcd_panel_handle_t panel, int width, int height,
                                   int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                                   bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y,
                    swap_xy) {}

void OttoEmojiDisplay::SetupUI() {
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI called more than once");
        return;
    }

    SpiLcdDisplay::SetupUI();

    auto* dark_theme = LvglThemeManager::GetInstance().GetTheme("dark");
    if (dark_theme != nullptr) {
        SetTheme(dark_theme);
    }

    {
        DisplayLockGuard lock(this);
        if (preview_image_ != nullptr) {
            lv_obj_set_size(preview_image_, width_, height_);
        }
    }

    SetEmotion("neutral");
}
