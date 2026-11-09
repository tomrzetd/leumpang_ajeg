// "menuconfig" Bluepad32 untuk Pico 2 W (Leumpang Radio)
#pragma once
#define CONFIG_BLUEPAD32_MAX_DEVICES 2                 // satu stik + mouse virtual (touchpad)
#define CONFIG_BLUEPAD32_MAX_ALLOWLIST 4
#define CONFIG_BLUEPAD32_GAP_SECURITY 1
#define CONFIG_BLUEPAD32_ENABLE_BLE_BY_DEFAULT 1      // DualSense/Xbox BLE juga bisa
#define CONFIG_BLUEPAD32_ENABLE_VIRTUAL_DEVICE_BY_DEFAULT 1   // klik touchpad DS4/DS5 = E-STOP

#define CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#define CONFIG_TARGET_PICO_W

#define CONFIG_BLUEPAD32_LOG_LEVEL 2                   // info
