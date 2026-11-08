// Bus servo ICS 3.5 di USART2 (kanan) & USART6 (kiri): 1,25 Mbps 8E1, FIFO 8 byte
// Akses register langsung (polling) - dipanggil dari ISR kontrol TIM6.
#include "board.h"
#include "app.h"

namespace {
struct Bus { USART_TypeDef* u; GPIO_TypeDef* enPort; uint16_t enPin; };
const Bus BUS[2] = { { ICS_R_UART, ICS_R_EN_PORT, ICS_R_EN_PIN }, { ICS_L_UART, ICS_L_EN_PORT, ICS_L_EN_PIN } };
UART_HandleTypeDef s_h[2];

inline void dirTx(int b, bool tx) {
    HAL_GPIO_WritePin(BUS[b].enPort, BUS[b].enPin, (tx == (ICS_EN_TX_LEVEL != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void pinAf(GPIO_TypeDef* port, uint16_t pin, uint8_t af, uint32_t pull) {
    GPIO_InitTypeDef g = {};
    g.Pin = pin; g.Mode = GPIO_MODE_AF_PP; g.Pull = pull; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = af;
    HAL_GPIO_Init(port, &g);
}
}  // namespace

void Stm32IcsBus::init() {
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE(); __HAL_RCC_USART6_CLK_ENABLE();
    pinAf(ICS_R_TX_PORT, ICS_R_TX_PIN, ICS_R_AF, GPIO_NOPULL);
    pinAf(ICS_R_RX_PORT, ICS_R_RX_PIN, ICS_R_AF, GPIO_PULLUP);
    pinAf(ICS_L_TX_PORT, ICS_L_TX_PIN, ICS_L_AF, GPIO_NOPULL);
    pinAf(ICS_L_RX_PORT, ICS_L_RX_PIN, ICS_L_AF, GPIO_PULLUP);
    for (int b = 0; b < 2; ++b) {
        GPIO_InitTypeDef g = {};
        g.Pin = BUS[b].enPin; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BUS[b].enPort, &g);
        dirTx(b, false);
        UART_HandleTypeDef& h = s_h[b];
        h.Instance = BUS[b].u;
        h.Init.BaudRate = la::ICS_BAUD;
        h.Init.WordLength = UART_WORDLENGTH_9B;    // 8 data + paritas
        h.Init.StopBits = UART_STOPBITS_1;
        h.Init.Parity = UART_PARITY_EVEN;
        h.Init.Mode = UART_MODE_TX_RX;
        h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        h.Init.OverSampling = UART_OVERSAMPLING_16;   // 250 MHz / 1,25 MHz = 200 (pas)
        h.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        h.Init.ClockPrescaler = UART_PRESCALER_DIV1;
        HAL_UART_Init(&h);
        HAL_UARTEx_EnableFifoMode(&h);               // 8 byte RX FIFO: aman bila ISR IMU menyela
    }
}

void Stm32IcsBus::exchange(la::IcsExchange& x) {
    for (int b = 0; b < 2; ++b) {
        USART_TypeDef* u = BUS[b].u;
        while (u->ISR & USART_ISR_RXNE_RXFNE) (void)u->RDR;
        u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_PECF | USART_ICR_NECF | USART_ICR_TCCF;
        dirTx(b, true);
        x.got[b] = false;
    }
    for (int i = 0; i < 3; ++i)
        for (int b = 0; b < 2; ++b) {
            while (!(BUS[b].u->ISR & USART_ISR_TXE_TXFNF)) {}
            BUS[b].u->TDR = x.tx[b][i];
        }
    const int64_t txDeadline = la_micros() + 100;
    for (int b = 0; b < 2; ++b) {
        while (!(BUS[b].u->ISR & USART_ISR_TC) && la_micros() < txDeadline) {}
        dirTx(b, false);
    }
    uint8_t rx[2][12];
    uint8_t n[2] = {0, 0};
    const int64_t deadline = la_micros() + ICS_REPLY_TIMEOUT_US;
    while (!(x.got[0] && x.got[1]) && la_micros() < deadline) {
        for (int b = 0; b < 2; ++b) {
            USART_TypeDef* u = BUS[b].u;
            if (u->ISR & USART_ISR_ORE) u->ICR = USART_ICR_ORECF;
            while (!x.got[b] && (u->ISR & USART_ISR_RXNE_RXFNE) && n[b] < sizeof(rx[b])) {
                rx[b][n[b]++] = (uint8_t)u->RDR;
                x.got[b] = la::ics::parseReply(rx[b], n[b], x.tx[b], &x.pos[b]);
            }
        }
    }
}
