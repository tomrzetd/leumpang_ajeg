// =============================================================================
//  Leumpang Radio - ESP32 klasik (WROOM-32 DevKit / PICO-D4), Bluepad32 + BTstack
//  UART2: TX=GPIO26 -> RX robot, RX=GPIO27 <- TX robot, 460800 8N1, 3,3 V, GND bersama.
//  (ubah lewat idf.py menuconfig -> Leumpang Radio). Log: UART0/USB seperti biasa.
// =============================================================================
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "radio_hal.h"
#include "sdkconfig.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Leumpang Radio memakai BLUEPAD32_PLATFORM_CUSTOM"
#endif
#if !CONFIG_IDF_TARGET_ESP32
#error "Hanya ESP32 klasik yang punya Bluetooth Classic (DualShock 4). Pakai: idf.py set-target esp32"
#endif

namespace {
constexpr uart_port_t RADIO_PORT = UART_NUM_2;
constexpr int RADIO_BAUD = 460800;
}

extern "C" {
void radio_hal_uart_write(const uint8_t* d, size_t n) { uart_write_bytes(RADIO_PORT, d, n); }
int radio_hal_uart_read(uint8_t* d, size_t max) {
    size_t avail = 0;
    if (uart_get_buffered_data_len(RADIO_PORT, &avail) != ESP_OK || avail == 0) return 0;
    return uart_read_bytes(RADIO_PORT, d, (uint32_t)(avail < max ? avail : max), 0);
}
void radio_hal_led(int on) {
    if (CONFIG_LA_RADIO_LED >= 0) gpio_set_level((gpio_num_t)CONFIG_LA_RADIO_LED, on ? 1 : 0);
}
uint32_t radio_hal_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
const char* radio_hal_board(void) { return "esp32"; }

void app_main(void) {
    uart_config_t c = {};
    c.baud_rate = RADIO_BAUD;
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(RADIO_PORT, 1024, 1024, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(RADIO_PORT, &c));
    ESP_ERROR_CHECK(uart_set_pin(RADIO_PORT, CONFIG_LA_RADIO_TX, CONFIG_LA_RADIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (CONFIG_LA_RADIO_LED >= 0) {
        gpio_reset_pin((gpio_num_t)CONFIG_LA_RADIO_LED);
        gpio_set_direction((gpio_num_t)CONFIG_LA_RADIO_LED, GPIO_MODE_OUTPUT);
    }

#if defined(CONFIG_ESP_CONSOLE_UART) && !defined(CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE)
    btstack_stdio_init();
#endif
    btstack_init();
    uni_platform_set_custom(radio_platform());
    uni_init(0, nullptr);
    btstack_run_loop_execute();            // tidak kembali
}
}
