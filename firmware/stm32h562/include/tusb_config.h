/* TinyUSB: perangkat CDC-ACM tunggal (konsol + LAP) di USB FS STM32H5 */
#pragma once
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU          OPT_MCU_STM32H5
#endif
#define CFG_TUSB_OS           OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_ENABLED       1
#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_CDC           1
#define CFG_TUD_MSC           0
#define CFG_TUD_HID           0
#define CFG_TUD_MIDI          0
#define CFG_TUD_VENDOR        0
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 2048
#define CFG_TUD_CDC_EP_BUFSIZE 64
