#include "mbs_encoder.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include <LibPrintf.h>
#include "pin_defs.h"

USART_TypeDef *UartEncBase;
volatile uint8_t rx_buffer_mbs[10];

mbs_data_t mbs_data = {0};
volatile uint8_t mbs_link_err = MBS_ERR_TIMEOUT; // до первой удачной транзакции - "нет связи"

void enc_dma_init(HardwareSerial &Serial_enc) {
    UartEncBase = (USART_TypeDef *)Serial_enc.getHandle()->Instance;

    if (UartEncBase == USART1) NVIC_DisableIRQ(USART1_IRQn);
    else if (UartEncBase == USART2) NVIC_DisableIRQ(USART2_IRQn);
    else if (UartEncBase == USART3) NVIC_DisableIRQ(USART3_IRQn);

    UartEncBase->CR1 &= ~(USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE);

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART1_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
}

// Одна транзакция запрос-ответ по RS485: отправка команды, приём ответа по DMA,
// проверка CRC. true - в rx_buffer_mbs лежит валидный ответ на cmd.
static bool mbs_request(uint8_t cmd) {
    uint8_t crc_pos = Get_CRC_Position(cmd);
    if (crc_pos == 0) return false; // неизвестная команда
    uint8_t expected_payload = crc_pos + 1;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    UartEncBase->CR3 &= ~USART_CR3_DMAR;

    UartEncBase->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_TCCF;
    while (UartEncBase->ISR & USART_ISR_RXNE_RXFNE) {
        (void)UartEncBase->RDR;
    }

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&UartEncBase->RDR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)rx_buffer_mbs);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, expected_payload);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    UartEncBase->CR3 |= USART_CR3_DMAR;

    // Используем BSRR для мгновенного переключения (работает для PB9)
    GPIOB->BSRR = GPIO_BSRR_BS9;
    UartEncBase->TDR = cmd;
    while (!(UartEncBase->ISR & USART_ISR_TC));
    GPIOB->BSRR = GPIO_BSRR_BR9;

    uint32_t timeout = micros();
    while (LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1) > 0) {
        if (micros() - timeout > 2000) break;
    }

    uint8_t received = expected_payload - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);

    if (received != expected_payload) {
        // RX Timeout
        mbs_link_err = MBS_ERR_TIMEOUT;
        return false;
    }

    uint8_t crc_calc = Calc_CRC8((uint8_t*)rx_buffer_mbs, crc_pos);
    if (crc_calc != rx_buffer_mbs[crc_pos]) {
        // CRC Err
        digital_io_toggle(LED1_GPIO, LED1_pin);
        mbs_link_err = MBS_ERR_CRC;
        return false;
    }
    mbs_link_err = MBS_ERR_NONE;
    return true;
}

float readMySensorCallback() {
    if (mbs_request(MBS_CMD_GET_POS_RPM)) {
        mbs_data.angle = Get_angle_radian(rx_buffer_mbs);
        mbs_data.rpm   = Get_RPM_Raw(rx_buffer_mbs);
    }
    return mbs_data.angle;
}

float Get_RPM(void) {
    if (mbs_request(MBS_CMD_GET_POS_RPM)) {
        mbs_data.angle = Get_angle_radian(rx_buffer_mbs);
        mbs_data.rpm   = Get_RPM_Raw(rx_buffer_mbs);
    }
    return mbs_data.rpm;
}

float MBS_GetTemperature(void) {
    if (mbs_request(MBS_CMD_GET_POS_TEMP)) {
        mbs_data.angle = Get_angle_radian(rx_buffer_mbs);
        mbs_data.temp  = Get_Temperature(rx_buffer_mbs);
    }
    return mbs_data.temp;
}

uint8_t MBS_GetStatus(void) {
    if (mbs_request(MBS_CMD_GET_POS_STATUS)) {
        mbs_data.angle  = Get_angle_radian(rx_buffer_mbs);
        mbs_data.status = Get_Status_Raw(rx_buffer_mbs);
    }
    return mbs_data.status;
}
