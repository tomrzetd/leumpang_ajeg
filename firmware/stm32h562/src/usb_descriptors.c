/* Deskriptor USB: satu fungsi CDC-ACM (konsol teks + protokol LAP)
 * VID/PID 0x1209/0x0001 = pid.codes "test PID" - ganti bila didistribusikan. */
#include <string.h>
#include "tusb.h"
#include "board.h"

static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE, .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC, .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, .idVendor = 0x1209, .idProduct = 0x0001, .bcdDevice = 0x0100,
    .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3, .bNumConfigurations = 1,
};
uint8_t const* tud_descriptor_device_cb(void) { return (uint8_t const*)&desc_device; }

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_cfg[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) { (void)index; return desc_cfg; }

static char const* string_desc[] = { (const char[]){0x09, 0x04}, "Leumpang Ajeg", "Leumpang Ajeg STM32H562", NULL, "LAP CDC" };
static uint16_t desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t n;
    if (index == 0) { memcpy(&desc_str[1], string_desc[0], 2); n = 1; }
    else {
        char serial[25];
        const char* s;
        if (index == 3) {                         /* nomor seri dari UID chip */
            const uint32_t* uid = (const uint32_t*)UID_BASE;
            static const char hx[] = "0123456789ABCDEF";
            for (int i = 0; i < 24; ++i) serial[i] = hx[(uid[i / 8] >> (28 - 4 * (i % 8))) & 0xF];
            serial[24] = 0; s = serial;
        } else if (index < sizeof(string_desc) / sizeof(string_desc[0])) s = string_desc[index];
        else return NULL;
        n = strlen(s); if (n > 32) n = 32;
        for (size_t i = 0; i < n; ++i) desc_str[1 + i] = (uint8_t)s[i];
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return desc_str;
}
