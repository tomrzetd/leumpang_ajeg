// Clock sistem & jam mikrodetik (STM32H562)
#include "board.h"
#include "app.h"

void la_clock_init() {
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);      // wajib untuk 250 MHz
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitTypeDef o = {};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
    o.HSEState = RCC_HSE_ON;
    o.HSI48State = RCC_HSI48_ON;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
    o.PLL.PLLM = 2;                              // 8 MHz / 2 = 4 MHz
    o.PLL.PLLN = 125;                            // VCO 500 MHz
    o.PLL.PLLP = 2;                              // SYSCLK 250 MHz
    o.PLL.PLLQ = 5;                              // 100 MHz -> SDMMC1
    o.PLL.PLLR = 2;
    o.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;          // 4..8 MHz
    o.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
    o.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) { while (1) {} }

    RCC_ClkInitTypeDef c = {};
    c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV1;            // 250 MHz: USART2/6, I2C1, TIM2/6/7
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    c.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) { while (1) {} }
    __HAL_RCC_PLL1_CLKOUT_ENABLE(RCC_PLL1_DIVQ);

    RCC_PeriphCLKInitTypeDef p = {};
    p.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_SDMMC1;
    p.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    p.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLL1Q;
    HAL_RCCEx_PeriphCLKConfig(&p);

    // HSI48 dikunci ke SOF USB (akurasi 48 MHz tanpa kristal tambahan)
    __HAL_RCC_CRS_CLK_ENABLE();
    RCC_CRSInitTypeDef crs = {};
    crs.Prescaler = RCC_CRS_SYNC_DIV1;
    crs.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
    HAL_RCCEx_CRSConfig(&crs);

    HAL_ICACHE_Enable();
}

// ---- jam mikrodetik: TIM2 32-bit @ 1 MHz + word atas dari ISR overflow ----
volatile uint32_t g_tim2_hi = 0;

void la_timebase_init() {
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0;
    TIM2->PSC = (uint32_t)(HAL_RCC_GetPCLK1Freq() / 1000000u) - 1u;   // timer clock = PCLK1 (APB /1)
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;                       // muat PSC
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM2_IRQn, PRIO_TIMEBASE, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
}

extern "C" void la_tim2_overflow_isr(void) {
    if (TIM2->SR & TIM_SR_UIF) { TIM2->SR = ~TIM_SR_UIF; g_tim2_hi = g_tim2_hi + 1; }
}

int64_t la_micros() {
    uint32_t hi, lo;
    do {
        hi = g_tim2_hi;
        lo = TIM2->CNT;
        // overflow terjadi tapi ISR belum jalan (mis. dipanggil dari ISR prioritas lebih tinggi)
        if ((TIM2->SR & TIM_SR_UIF) && lo < 0x80000000u) { ++hi; break; }
    } while (hi != g_tim2_hi);
    return ((int64_t)hi << 32) | lo;
}
