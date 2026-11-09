/* =============================================================================
 *  Board: WeAct STM32H562RGT6 core board (LQFP64, HSE 8 MHz, LSE 32.768 kHz)
 *  On-board: LED PB2, KEY PC13, USB-C PA11/PA12, microSD SDMMC1, SWD, USART1 PA9/PA10
 *
 *  Pin robot (VERIFIKASI terhadap datasheet STM32H562 tabel alternate function
 *  dan header board sebelum menyambung):
 *    Bus ICS kanan : USART2 TX PA2, RX PA3 (AF7), EN PA4
 *    Bus ICS kiri  : USART6 TX PC6, RX PC7 (AF7), EN PB0
 *    IMU ICM-20948 : I2C1 SCL PB6, SDA PB7 (AF4), 400 kHz
 *    Pin ukur T2   : PB1  (HIGH selama satu siklus kontrol)
 *    microSD       : SDMMC1 D0..D3 PC8..PC11, CLK PC12, CMD PD2 (AF12)
 *    USB CDC       : PA11/PA12 (AF10) - konsol + LAP (HP OTG / PC)
 *  Belum di Fase 1: LCD ST7789, RGB WS2812 (lihat docs/ROADMAP.md).
 * ============================================================================= */
#pragma once
#include "stm32h5xx_hal.h"

#define LA_BOARD_NAME        "leumpang-h562"

#define LED_PORT  GPIOB
#define LED_PIN   GPIO_PIN_2
#define T2_PORT   GPIOB
#define T2_PIN    GPIO_PIN_1

#define ICS_R_UART        USART2
#define ICS_R_TX_PORT     GPIOA
#define ICS_R_TX_PIN      GPIO_PIN_2
#define ICS_R_RX_PORT     GPIOA
#define ICS_R_RX_PIN      GPIO_PIN_3
#define ICS_R_AF          GPIO_AF7_USART2
#define ICS_R_EN_PORT     GPIOA
#define ICS_R_EN_PIN      GPIO_PIN_4

#define ICS_L_UART        USART6
#define ICS_L_TX_PORT     GPIOC
#define ICS_L_TX_PIN      GPIO_PIN_6
#define ICS_L_RX_PORT     GPIOC
#define ICS_L_RX_PIN      GPIO_PIN_7
#define ICS_L_AF          GPIO_AF7_USART6
#define ICS_L_EN_PORT     GPIOB
#define ICS_L_EN_PIN      GPIO_PIN_0

#define ICS_EN_TX_LEVEL   1          /* EN HIGH = TX (74HC126 / driver Kondo) */
#define ICS_REPLY_TIMEOUT_US 300

#define IMU_I2C           I2C1
/* Leumpang Radio (Fase 4B): USART1 PA9 TX -> RX radio, PA10 RX <- TX radio, 460800 8N1 */
#define RADIO_UART        USART1
#define RADIO_BAUD        460800u

#define IMU_SCL_PORT      GPIOB
#define IMU_SCL_PIN       GPIO_PIN_6
#define IMU_SDA_PORT      GPIOB
#define IMU_SDA_PIN       GPIO_PIN_7
#define IMU_AF            GPIO_AF4_I2C1
/* TIMINGR 400 kHz @ PCLK1 250 MHz: PRESC=5 SCLDEL=4 SDADEL=2 SCLH=0x22 SCLL=0x3D (~390 kHz).
   Dihitung manual; verifikasi dengan osiloskop / STM32CubeMX. */
#define IMU_I2C_TIMING    0x5042223DU

/* Prioritas NVIC (angka kecil = lebih tinggi). SysTick 0 (HAL timeout dipakai di ISR). */
#define PRIO_TIMEBASE     1          /* TIM2 overflow jam mikrodetik */
#define PRIO_RADIO        2          /* USART1 RX: hanya salin byte ke ring (us); di atas kontrol agar FIFO tak luber */
#define PRIO_USB          3
#define PRIO_IMU          5          /* TIM7 500 Hz: baca I2C + Mahony */
#define PRIO_CONTROL      6          /* TIM6 100 Hz: la::Controller::step */
