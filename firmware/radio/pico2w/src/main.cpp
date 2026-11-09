// =============================================================================
//  Leumpang Radio - Raspberry Pi Pico 2 W (RP2350 + CYW43439, Bluepad32)
//  UART0: GP0 TX -> RX robot, GP1 RX <- TX robot, 460800 8N1, 3,3 V, GND bersama.
//  Log: USB (stdio). LED: LED on-board (lewat CYW43).
// =============================================================================
#include <btstack_run_loop.h>
#include <hardware/uart.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>
#include <uni.h>

#include "radio_hal.h"
#include "sdkconfig.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W harus memakai BLUEPAD32_PLATFORM_CUSTOM"
#endif

#ifndef LA_RADIO_UART_TX
#define LA_RADIO_UART_TX 0
#endif
#ifndef LA_RADIO_UART_RX
#define LA_RADIO_UART_RX 1
#endif
static constexpr uint32_t RADIO_BAUD = 460800;
#define RADIO_UART uart0

extern "C" {
void radio_hal_uart_write(const uint8_t* d, size_t n) { uart_write_blocking(RADIO_UART, d, n); }
int radio_hal_uart_read(uint8_t* d, size_t max) {
    size_t n = 0;
    while (n < max && uart_is_readable(RADIO_UART)) d[n++] = (uint8_t)uart_getc(RADIO_UART);
    return (int)n;
}
void radio_hal_led(int on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on != 0); }
uint32_t radio_hal_ms(void) { return to_ms_since_boot(get_absolute_time()); }
const char* radio_hal_board(void) { return "pico2w"; }
}

int main() {
    stdio_init_all();
    uart_init(RADIO_UART, RADIO_BAUD);
    gpio_set_function(LA_RADIO_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(LA_RADIO_UART_RX, GPIO_FUNC_UART);
    gpio_pull_up(LA_RADIO_UART_RX);
    uart_set_fifo_enabled(RADIO_UART, true);

    if (cyw43_arch_init()) {                  // juga menyalakan Bluetooth (CYW43_ENABLE_BLUETOOTH)
        loge("radio: cyw43 gagal\n");
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    uni_platform_set_custom(radio_platform());
    uni_init(0, nullptr);
    btstack_run_loop_execute();               // tidak kembali
    return 0;
}
