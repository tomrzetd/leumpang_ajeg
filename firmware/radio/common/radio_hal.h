// =============================================================================
//  Leumpang Radio - antarmuka HAL kecil per board (ESP32 klasik / Pico 2 W).
//  Semua dipanggil dari run loop BTstack (satu thread).
// =============================================================================
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void        radio_hal_uart_write(const uint8_t* d, size_t n);     // ke robot (blocking singkat)
int         radio_hal_uart_read(uint8_t* d, size_t max);          // dari robot, non-blocking
void        radio_hal_led(int on);                                // LED status board
uint32_t    radio_hal_ms(void);
const char* radio_hal_board(void);

// Platform Bluepad32 untuk Leumpang (radio.cpp). Panggil uni_platform_set_custom() dengan ini.
struct uni_platform* radio_platform(void);

#ifdef __cplusplus
}
#endif
