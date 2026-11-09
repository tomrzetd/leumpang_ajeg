// =============================================================================
//  Leumpang Radio (Fase 4B) di USART1 (PA9/PA10), 460800 8N1.
//  ISR RX hanya menyalin byte ke ring (prioritas di atas kontrol supaya FIFO 8 byte
//  tidak luber saat ISR kontrol menunggu bus ICS); dekode LAP di main loop.
// =============================================================================
#include "app.h"
#include "board.h"
#include "stm32h5xx_hal.h"

namespace {
constexpr uint16_t RING = 1024;                 // pangkat dua
uint8_t s_ring[RING];
volatile uint16_t s_head = 0, s_tail = 0;
la::lap::LapEndpoint s_ep;
bool s_on = false;

void radioWrite(void*, const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        uint32_t guard = 0;
        while (!(RADIO_UART->ISR & USART_ISR_TXE_TXFNF)) if (++guard > 100000u) return;   // radio mati: jangan macet
        RADIO_UART->TDR = d[i];
    }
}
}  // namespace

extern "C" void USART1_IRQHandler(void) {
    const uint32_t isr = RADIO_UART->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
        RADIO_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    while (RADIO_UART->ISR & USART_ISR_RXNE_RXFNE) {
        const uint8_t b = (uint8_t)RADIO_UART->RDR;
        const uint16_t next = (uint16_t)((s_head + 1) & (RING - 1));
        if (next != s_tail) { s_ring[s_head] = b; s_head = next; }
    }
}

void radio_init(const la::lap::LapEndpoint::Host& host, const la::lap::Hello& self) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef g = {};
    g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);
    RADIO_UART->CR1 = 0;
    RADIO_UART->BRR = (HAL_RCC_GetPCLK2Freq() + RADIO_BAUD / 2) / RADIO_BAUD;
    RADIO_UART->CR2 = 0;
    RADIO_UART->CR3 = 0;
    RADIO_UART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
    HAL_NVIC_SetPriority(USART1_IRQn, PRIO_RADIO, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    la::lap::LapEndpoint::Host h = host;
    h.write = radioWrite; h.writeCtx = nullptr;
    s_ep.init(h, self);
    s_on = true;
}

void radio_poll(uint32_t nowMs) {
    if (!s_on) return;
    uint8_t buf[64];
    size_t n = 0;
    while (s_tail != s_head && n < sizeof buf) {
        buf[n++] = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1) & (RING - 1));
    }
    if (n) s_ep.onBytes(buf, n, nowMs);
    s_ep.poll(nowMs);
}
