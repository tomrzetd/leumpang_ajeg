/* FatFs disk I/O untuk microSD on-board (SDMMC1 4-bit, polling, HW flow control) */
#include "board.h"
#include "ff.h"
#include "diskio.h"

SD_HandleTypeDef g_hsd;
static volatile DSTATUS s_stat = STA_NOINIT;

void HAL_SD_MspInit(SD_HandleTypeDef* h) {
    (void)h;
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &g);
}

static int waitReady(uint32_t timeoutMs) {
    const uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&g_hsd) != HAL_SD_CARD_TRANSFER)
        if (HAL_GetTick() - t0 > timeoutMs) return 0;
    return 1;
}

DSTATUS disk_status(BYTE pdrv) { return pdrv ? STA_NOINIT : s_stat; }

DSTATUS disk_initialize(BYTE pdrv) {
    if (pdrv) return STA_NOINIT;
    g_hsd.Instance = SDMMC1;
    g_hsd.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    g_hsd.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    g_hsd.Init.BusWide = SDMMC_BUS_WIDE_4B;
    g_hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    g_hsd.Init.ClockDiv = 2;                  /* 100 MHz / (2*2) = 25 MHz */
    s_stat = (HAL_SD_Init(&g_hsd) == HAL_OK) ? 0 : STA_NOINIT;
    return s_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
    if (pdrv || (s_stat & STA_NOINIT)) return RES_NOTRDY;
    if (HAL_SD_ReadBlocks(&g_hsd, buff, (uint32_t)sector, count, 1000) != HAL_OK) return RES_ERROR;
    return waitReady(1000) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
    if (pdrv || (s_stat & STA_NOINIT)) return RES_NOTRDY;
    if (HAL_SD_WriteBlocks(&g_hsd, buff, (uint32_t)sector, count, 1000) != HAL_OK) return RES_ERROR;
    return waitReady(1000) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
    if (pdrv || (s_stat & STA_NOINIT)) return RES_NOTRDY;
    HAL_SD_CardInfoTypeDef ci;
    switch (cmd) {
        case CTRL_SYNC: return waitReady(1000) ? RES_OK : RES_ERROR;
        case GET_SECTOR_COUNT: HAL_SD_GetCardInfo(&g_hsd, &ci); *(LBA_t*)buff = ci.LogBlockNbr; return RES_OK;
        case GET_SECTOR_SIZE:  HAL_SD_GetCardInfo(&g_hsd, &ci); *(WORD*)buff = (WORD)ci.LogBlockSize; return RES_OK;
        case GET_BLOCK_SIZE:   HAL_SD_GetCardInfo(&g_hsd, &ci); *(DWORD*)buff = ci.LogBlockSize / 512; return RES_OK;
        default: return RES_PARERR;
    }
}
