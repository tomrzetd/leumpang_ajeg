// stub sdkconfig: S3 default, -DLA_STUB_P4 untuk varian ESP32-P4
#pragma once
#ifdef LA_STUB_P4
#define CONFIG_IDF_TARGET_ESP32P4 1
#define CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG 1
#else
#define CONFIG_IDF_TARGET_ESP32S3 1
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#endif
