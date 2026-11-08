/* Handler interrupt STM32H562 */
#include "board.h"
#include "app.h"
#include "tusb.h"

void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

void TIM2_IRQHandler(void) { la_tim2_overflow_isr(); }
void TIM6_IRQHandler(void) { if (TIM6->SR & TIM_SR_UIF) { TIM6->SR = ~TIM_SR_UIF; la_tim6_isr(); } }
void TIM7_IRQHandler(void) { if (TIM7->SR & TIM_SR_UIF) { TIM7->SR = ~TIM_SR_UIF; la_tim7_isr(); } }
void USB_DRD_FS_IRQHandler(void) { tud_int_handler(0); }

uint32_t tusb_time_millis_api(void) { return HAL_GetTick(); }
