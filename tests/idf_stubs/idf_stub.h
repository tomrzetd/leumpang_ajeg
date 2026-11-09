// Stub API ESP-IDF v5.3 untuk syntax-check host (signature mengikuti header IDF).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NVS_NO_FREE_PAGES 0x110d
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1110
#define ESP_ERROR_CHECK(x) do { (void)(x); } while (0)
#define IRAM_ATTR
// ---- FreeRTOS ----
typedef uint32_t TickType_t; typedef int BaseType_t; typedef unsigned UBaseType_t; typedef uint8_t StackType_t;
typedef void* TaskHandle_t; typedef void* QueueHandle_t; typedef void* SemaphoreHandle_t;
typedef void (*TaskFunction_t)(void*);
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
typedef struct { int owner; int count; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0,0}
void vPortEnterCritical(portMUX_TYPE*); void vPortExitCritical(portMUX_TYPE*);
#define portENTER_CRITICAL(m) vPortEnterCritical(m)
#define portEXIT_CRITICAL(m) vPortExitCritical(m)
#define portYIELD_FROM_ISR() do{}while(0)
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, UBaseType_t, TaskHandle_t*, BaseType_t);
void vTaskDelete(TaskHandle_t); void vTaskDelay(TickType_t); BaseType_t xTaskDelayUntil(TickType_t*, TickType_t); TickType_t xTaskGetTickCount(void);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t*); uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
QueueHandle_t xQueueCreate(UBaseType_t, UBaseType_t);
BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t); BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t);
SemaphoreHandle_t xSemaphoreCreateBinary(void); SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t); BaseType_t xSemaphoreGive(SemaphoreHandle_t);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t, BaseType_t*);
// ---- esp_timer ----
int64_t esp_timer_get_time(void);
void esp_restart(void);
typedef void* esp_timer_handle_t;
typedef void (*esp_timer_cb_t)(void*);
typedef struct { esp_timer_cb_t callback; void* arg; int dispatch_method; const char* name; bool skip_unhandled_events; } esp_timer_create_args_t;
esp_err_t esp_timer_create(const esp_timer_create_args_t*, esp_timer_handle_t*);
esp_err_t esp_timer_start_once(esp_timer_handle_t, uint64_t); esp_err_t esp_timer_stop(esp_timer_handle_t);
// ---- GPIO ----
typedef int gpio_num_t;
typedef enum { GPIO_MODE_DISABLE=0, GPIO_MODE_INPUT=1, GPIO_MODE_OUTPUT=2 } gpio_mode_t;
typedef enum { GPIO_PULLUP_ONLY, GPIO_PULLDOWN_ONLY, GPIO_PULLUP_PULLDOWN, GPIO_FLOATING } gpio_pull_mode_t;
typedef enum { GPIO_INTR_DISABLE=0, GPIO_INTR_POSEDGE=1 } gpio_int_type_t;
typedef struct { uint64_t pin_bit_mask; gpio_mode_t mode; int pull_up_en; int pull_down_en; gpio_int_type_t intr_type; } gpio_config_t;
esp_err_t gpio_config(const gpio_config_t*); esp_err_t gpio_set_level(gpio_num_t, uint32_t);
esp_err_t gpio_set_pull_mode(gpio_num_t, gpio_pull_mode_t);
esp_err_t gpio_install_isr_service(int); typedef void (*gpio_isr_t)(void*);
esp_err_t gpio_isr_handler_add(gpio_num_t, gpio_isr_t, void*);
// ---- UART ----
typedef int uart_port_t;
#define UART_NUM_1 1
#define UART_NUM_2 2
#define UART_PIN_NO_CHANGE (-1)
typedef enum { UART_DATA_8_BITS = 3 } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE=0, UART_PARITY_EVEN=2 } uart_parity_t;
typedef enum { UART_STOP_BITS_1 = 1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE = 0 } uart_hw_flowcontrol_t;
typedef enum { UART_SCLK_DEFAULT = 4 } uart_sclk_t;
typedef enum { UART_MODE_UART=0, UART_MODE_RS485_HALF_DUPLEX=1 } uart_mode_t;
#define UART_SIGNAL_RTS_INV (1<<6)
typedef struct { int baud_rate; uart_word_length_t data_bits; uart_parity_t parity; uart_stop_bits_t stop_bits;
                 uart_hw_flowcontrol_t flow_ctrl; uint8_t rx_flow_ctrl_thresh; uart_sclk_t source_clk; } uart_config_t;
esp_err_t uart_driver_install(uart_port_t, int, int, int, QueueHandle_t*, int);
esp_err_t uart_param_config(uart_port_t, const uart_config_t*);
esp_err_t uart_set_pin(uart_port_t, int, int, int, int);
esp_err_t uart_set_mode(uart_port_t, uart_mode_t); esp_err_t uart_set_line_inverse(uart_port_t, uint32_t);
esp_err_t uart_set_rx_full_threshold(uart_port_t, int); esp_err_t uart_set_rx_timeout(uart_port_t, const uint8_t);
esp_err_t uart_flush_input(uart_port_t);
int uart_write_bytes(uart_port_t, const void*, size_t);
esp_err_t uart_wait_tx_idle_polling(uart_port_t);
esp_err_t uart_get_buffered_data_len(uart_port_t, size_t*);
int uart_read_bytes(uart_port_t, void*, uint32_t, TickType_t);
// ---- I2C master ----
typedef void* i2c_master_bus_handle_t; typedef void* i2c_master_dev_handle_t;
typedef int i2c_port_num_t;
#define I2C_NUM_0 0
typedef enum { I2C_CLK_SRC_DEFAULT = 0 } i2c_clock_source_t;
typedef enum { I2C_ADDR_BIT_LEN_7 = 0 } i2c_addr_bit_len_t;
typedef struct { i2c_port_num_t i2c_port; gpio_num_t sda_io_num; gpio_num_t scl_io_num; i2c_clock_source_t clk_source;
                 uint8_t glitch_ignore_cnt; int intr_priority; size_t trans_queue_depth;
                 struct { uint32_t enable_internal_pullup:1; uint32_t allow_pd:1; } flags; } i2c_master_bus_config_t;
typedef struct { i2c_addr_bit_len_t dev_addr_length; uint16_t device_address; uint32_t scl_speed_hz; uint32_t scl_wait_us;
                 struct { uint32_t disable_ack_check:1; } flags; } i2c_device_config_t;
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t*, i2c_master_bus_handle_t*);
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t*, i2c_master_dev_handle_t*);
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t);
esp_err_t i2c_master_probe(i2c_master_bus_handle_t, uint16_t, int);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t*, size_t, int);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t*, size_t, uint8_t*, size_t, int);
esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t);
// ---- heap ----
#define MALLOC_CAP_DMA (1<<3)
#define MALLOC_CAP_INTERNAL (1<<11)
#define MALLOC_CAP_SPIRAM (1<<10)
void* heap_caps_malloc(size_t, uint32_t); size_t heap_caps_get_free_size(uint32_t);
// ---- SPI / LCD ----
typedef enum { SPI2_HOST = 1 } spi_host_device_t;
#define SPI_DMA_CH_AUTO 3
typedef struct { union { int mosi_io_num; int data0_io_num; }; union { int miso_io_num; int data1_io_num; };
                 int sclk_io_num; union { int quadwp_io_num; int data2_io_num; }; union { int quadhd_io_num; int data3_io_num; };
                 int data4_io_num, data5_io_num, data6_io_num, data7_io_num; int max_transfer_sz; uint32_t flags; } spi_bus_config_t;
esp_err_t spi_bus_initialize(spi_host_device_t, const spi_bus_config_t*, int);
typedef void* esp_lcd_panel_io_handle_t; typedef void* esp_lcd_panel_handle_t; typedef void* esp_lcd_spi_bus_handle_t;
typedef struct { int dummy; } esp_lcd_panel_io_event_data_t;
typedef bool (*esp_lcd_panel_io_color_trans_done_cb_t)(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void*);
typedef struct { int cs_gpio_num; int dc_gpio_num; int spi_mode; unsigned int pclk_hz; size_t trans_queue_depth;
                 esp_lcd_panel_io_color_trans_done_cb_t on_color_trans_done; void* user_ctx; int lcd_cmd_bits; int lcd_param_bits;
                 struct { unsigned dc_low_on_data:1; } flags; } esp_lcd_panel_io_spi_config_t;
esp_err_t esp_lcd_new_panel_io_spi(esp_lcd_spi_bus_handle_t, const esp_lcd_panel_io_spi_config_t*, esp_lcd_panel_io_handle_t*);
typedef enum { LCD_RGB_ELEMENT_ORDER_RGB = 0 } lcd_rgb_element_order_t;
typedef struct { int reset_gpio_num; lcd_rgb_element_order_t rgb_ele_order; int data_endian; uint32_t bits_per_pixel;
                 struct { unsigned reset_active_high:1; } flags; void* vendor_config; } esp_lcd_panel_dev_config_t;
esp_err_t esp_lcd_new_panel_st7789(esp_lcd_panel_io_handle_t, const esp_lcd_panel_dev_config_t*, esp_lcd_panel_handle_t*);
esp_err_t esp_lcd_panel_reset(esp_lcd_panel_handle_t); esp_err_t esp_lcd_panel_init(esp_lcd_panel_handle_t);
esp_err_t esp_lcd_panel_invert_color(esp_lcd_panel_handle_t, bool); esp_err_t esp_lcd_panel_swap_xy(esp_lcd_panel_handle_t, bool);
esp_err_t esp_lcd_panel_mirror(esp_lcd_panel_handle_t, bool, bool); esp_err_t esp_lcd_panel_set_gap(esp_lcd_panel_handle_t, int, int);
esp_err_t esp_lcd_panel_disp_on_off(esp_lcd_panel_handle_t, bool);
esp_err_t esp_lcd_panel_draw_bitmap(esp_lcd_panel_handle_t, int, int, int, int, const void*);
// ---- RMT ----
typedef void* rmt_channel_handle_t; typedef void* rmt_encoder_handle_t;
typedef enum { RMT_CLK_SRC_DEFAULT = 0 } rmt_clock_source_t;
typedef struct { gpio_num_t gpio_num; rmt_clock_source_t clk_src; uint32_t resolution_hz; size_t mem_block_symbols;
                 size_t trans_queue_depth; int intr_priority; struct { uint32_t invert_out:1; uint32_t with_dma:1; } flags; } rmt_tx_channel_config_t;
typedef union { struct { uint16_t duration0:15; uint16_t level0:1; uint16_t duration1:15; uint16_t level1:1; }; uint32_t val; } rmt_symbol_word_t;
typedef struct { rmt_symbol_word_t bit0; rmt_symbol_word_t bit1; struct { uint32_t msb_first:1; } flags; } rmt_bytes_encoder_config_t;
typedef struct { int loop_count; struct { uint32_t eot_level:1; } flags; } rmt_transmit_config_t;
esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t*, rmt_channel_handle_t*);
esp_err_t rmt_new_bytes_encoder(const rmt_bytes_encoder_config_t*, rmt_encoder_handle_t*);
esp_err_t rmt_enable(rmt_channel_handle_t);
esp_err_t rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void*, size_t, const rmt_transmit_config_t*);
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t, int);
// ---- NVS ----
typedef uint32_t nvs_handle_t;
typedef enum { NVS_READONLY, NVS_READWRITE } nvs_open_mode_t;
esp_err_t nvs_flash_init(void); esp_err_t nvs_flash_erase(void);
esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t*); void nvs_close(nvs_handle_t);
esp_err_t nvs_set_blob(nvs_handle_t, const char*, const void*, size_t); esp_err_t nvs_get_blob(nvs_handle_t, const char*, void*, size_t*);
esp_err_t nvs_set_str(nvs_handle_t, const char*, const char*); esp_err_t nvs_get_str(nvs_handle_t, const char*, char*, size_t*);
esp_err_t nvs_commit(nvs_handle_t);
uint32_t esp_rom_crc32_le(uint32_t, uint8_t const*, uint32_t);
// ---- netif / wifi / event ----
typedef struct esp_netif_obj esp_netif_t;
typedef struct { uint32_t addr; } esp_ip4_addr_t;
typedef struct { esp_ip4_addr_t ip, netmask, gw; } esp_netif_ip_info_t;
typedef struct { int if_index; esp_netif_t* esp_netif; esp_netif_ip_info_t ip_info; bool ip_changed; } ip_event_got_ip_t;
#define IPSTR "%d.%d.%d.%d"
#define IP2STR(a) (int)((a)->addr & 0xff), (int)(((a)->addr >> 8) & 0xff), (int)(((a)->addr >> 16) & 0xff), (int)(((a)->addr >> 24) & 0xff)
esp_err_t esp_netif_init(void); esp_netif_t* esp_netif_create_default_wifi_ap(void); esp_netif_t* esp_netif_create_default_wifi_sta(void);
esp_err_t esp_netif_get_ip_info(esp_netif_t*, esp_netif_ip_info_t*);
typedef const char* esp_event_base_t;
extern esp_event_base_t const WIFI_EVENT; extern esp_event_base_t const IP_EVENT;
#define ESP_EVENT_ANY_ID (-1)
typedef void (*esp_event_handler_t)(void*, esp_event_base_t, int32_t, void*);
typedef void* esp_event_handler_instance_t;
esp_err_t esp_event_loop_create_default(void);
esp_err_t esp_event_handler_instance_register(esp_event_base_t, int32_t, esp_event_handler_t, void*, esp_event_handler_instance_t*);
enum { WIFI_EVENT_STA_START = 2, WIFI_EVENT_STA_DISCONNECTED = 5 };
enum { IP_EVENT_STA_GOT_IP = 0 };
typedef struct { int dummy; } wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() { 0 }
typedef enum { WIFI_STORAGE_FLASH, WIFI_STORAGE_RAM } wifi_storage_t;
typedef enum { WIFI_MODE_APSTA = 3 } wifi_mode_t;
typedef enum { WIFI_IF_STA = 0, WIFI_IF_AP = 1 } wifi_interface_t;
typedef enum { WIFI_AUTH_WPA2_PSK = 3 } wifi_auth_mode_t;
typedef enum { WIFI_PS_NONE = 0 } wifi_ps_type_t;
typedef struct { bool capable; bool required; } wifi_pmf_config_t;
typedef struct { uint8_t ssid[32]; uint8_t password[64]; uint8_t ssid_len; uint8_t channel; wifi_auth_mode_t authmode;
                 uint8_t ssid_hidden; uint8_t max_connection; uint16_t beacon_interval; wifi_pmf_config_t pmf_cfg; } wifi_ap_config_t;
typedef struct { uint8_t ssid[32]; uint8_t password[64]; } wifi_sta_config_t;
typedef union { wifi_ap_config_t ap; wifi_sta_config_t sta; } wifi_config_t;
typedef struct { int num; } wifi_sta_list_t;
typedef struct { int8_t rssi; } wifi_ap_record_t;
esp_err_t esp_wifi_init(const wifi_init_config_t*); esp_err_t esp_wifi_set_storage(wifi_storage_t);
esp_err_t esp_wifi_set_mode(wifi_mode_t); esp_err_t esp_wifi_set_config(wifi_interface_t, wifi_config_t*);
esp_err_t esp_wifi_get_config(wifi_interface_t, wifi_config_t*); esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_set_ps(wifi_ps_type_t); esp_err_t esp_wifi_connect(void); esp_err_t esp_wifi_disconnect(void);
esp_err_t esp_wifi_ap_get_sta_list(wifi_sta_list_t*); esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t*);
typedef enum { ESP_MAC_WIFI_SOFTAP = 1 } esp_mac_type_t;
esp_err_t esp_read_mac(uint8_t*, esp_mac_type_t);
// ---- httpd ----
typedef void* httpd_handle_t;
typedef enum { HTTP_GET = 1, HTTP_POST = 3 } httpd_method_t;
typedef struct httpd_req { httpd_handle_t handle; int method; const char uri[513]; size_t content_len; void* aux; void* user_ctx; } httpd_req_t;
typedef struct { const char* uri; httpd_method_t method; esp_err_t (*handler)(httpd_req_t*); void* user_ctx; } httpd_uri_t;
typedef struct { unsigned task_priority; size_t stack_size; BaseType_t core_id; uint16_t server_port; uint16_t max_open_sockets;
                 uint16_t max_uri_handlers; bool lru_purge_enable; } httpd_config_t;
#define HTTPD_DEFAULT_CONFIG() { 5, 4096, 0x7fffffff, 80, 7, 8, false }
#define HTTPD_RESP_USE_STRLEN -1
#define HTTPD_SOCK_ERR_TIMEOUT -3
esp_err_t httpd_start(httpd_handle_t*, const httpd_config_t*); esp_err_t httpd_register_uri_handler(httpd_handle_t, const httpd_uri_t*);
esp_err_t httpd_resp_send(httpd_req_t*, const char*, ssize_t); esp_err_t httpd_resp_set_status(httpd_req_t*, const char*);
esp_err_t httpd_resp_set_type(httpd_req_t*, const char*); esp_err_t httpd_resp_set_hdr(httpd_req_t*, const char*, const char*);
esp_err_t httpd_resp_send_500(httpd_req_t*);
size_t httpd_req_get_hdr_value_len(httpd_req_t*, const char*); esp_err_t httpd_req_get_hdr_value_str(httpd_req_t*, const char*, char*, size_t);
int httpd_req_recv(httpd_req_t*, char*, size_t); esp_err_t httpd_query_key_value(const char*, const char*, char*, size_t);
// ---- mbedtls / mdns ----
int mbedtls_base64_encode(unsigned char*, size_t, size_t*, const unsigned char*, size_t);
esp_err_t mdns_init(void); esp_err_t mdns_hostname_set(const char*);
esp_err_t mdns_service_add(const char*, const char*, const char*, uint16_t, void*, size_t);
// ---- log / rom ----
typedef enum { ESP_LOG_NONE = 0, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO } esp_log_level_t;
void esp_log_level_set(const char*, esp_log_level_t);
int esp_rom_printf(const char*, ...);
// ---- USB-Serial-JTAG ----
typedef struct { uint32_t tx_buffer_size; uint32_t rx_buffer_size; } usb_serial_jtag_driver_config_t;
#define USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT() { 256, 256 }
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t*);
int usb_serial_jtag_read_bytes(void*, uint32_t, TickType_t);
int usb_serial_jtag_write_bytes(const void*, size_t, TickType_t);
// ---- SDMMC / FATFS ----
typedef void* sd_pwr_ctrl_handle_t;
typedef struct { int slot; int max_freq_khz; sd_pwr_ctrl_handle_t pwr_ctrl_handle; uint32_t flags; } sdmmc_host_t;
#define SDMMC_HOST_DEFAULT() { 1, 20000, 0, 0 }
#define SDMMC_HOST_SLOT_0 0
#define SDMMC_FREQ_DEFAULT 20000
#define SDMMC_FREQ_HIGHSPEED 40000
#define SDMMC_SLOT_FLAG_INTERNAL_PULLUP (1 << 0)
typedef struct { gpio_num_t clk, cmd, d0, d1, d2, d3; uint8_t width; uint32_t flags; } sdmmc_slot_config_t;
#define SDMMC_SLOT_CONFIG_DEFAULT() { -1, -1, -1, -1, -1, -1, 4, 0 }
typedef struct { char name[8]; } sdmmc_cid_t;
typedef struct { int capacity; int sector_size; } sdmmc_csd_t;
typedef struct { sdmmc_cid_t cid; sdmmc_csd_t csd; } sdmmc_card_t;
typedef struct { bool format_if_mount_failed; int max_files; size_t allocation_unit_size; } esp_vfs_fat_sdmmc_mount_config_t;
esp_err_t esp_vfs_fat_sdmmc_mount(const char*, const sdmmc_host_t*, const void*, const esp_vfs_fat_sdmmc_mount_config_t*, sdmmc_card_t**);
typedef struct { int ldo_chan_id; } sd_pwr_ctrl_ldo_config_t;
esp_err_t sd_pwr_ctrl_new_on_chip_ldo(const sd_pwr_ctrl_ldo_config_t*, sd_pwr_ctrl_handle_t*);
esp_err_t esp_wifi_get_mac(wifi_interface_t, uint8_t*);
#ifdef __cplusplus
}
#endif
