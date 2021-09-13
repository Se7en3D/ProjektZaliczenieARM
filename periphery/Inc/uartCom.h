/*
 * uartCom.h
 *
 *  Created on: 13 sie 2021
 *      Author: DanielD
 */

#ifndef PERIPHERY_INC_UARTCOM_H_
#define PERIPHERY_INC_UARTCOM_H_
#include <stdio.h>
#include "stm32f4xx_hal.h"
#define UARTCOM_BUF_MAXLEN 100 //Maksymalna wielkość bufora nadajnika STM32->PC
#define UARTCOM_NO_DATA_TO_POP 300
#define UARTCOM_TIME_TO_DELETE_BUFFOR_FOR_DATA 10 //Czas po którym należy wysłać/usunąć znajdujące się bity jeśli nie odebrano żadnych informacji


/**
  * @brief  Struktura bufora kołowego oraz bufora od dekodowania bitów transmisji
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope entire ADC (affects regular and injected groups): ClockPrescaler, Resolution, ScanConvMode, DataAlign, ScanConvMode, EOCSelection, LowPowerAutoWait, LowPowerAutoPowerOff, ChannelsBank.
  *          - Scope regular group: ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode, NbrOfDiscConversion, ExternalTrigConvEdge, ExternalTrigConv.
  * @note   The setting of these parameters with function HAL_ADC_Init() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all parameters except 'Resolution', 'ScanConvMode', 'DiscontinuousConvMode', 'NbrOfDiscConversion' : ADC enabled without conversion on going on regular group.
  *          - For parameters 'ExternalTrigConv' and 'ExternalTrigConvEdge': ADC enabled, even with conversion on going.
  *         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
  *         without error reporting (as it can be the expected behaviour in case of intended action to update another parameter (which fulfills the ADC state condition) on the fly).
  */
typedef struct {
    uint8_t buffer[UARTCOM_BUF_MAXLEN]; /*!< Bufor próbek o wielkości 8 bitów jego wielkość jest zalezna od parametru UARTCOM_BUF_MAXLEN*/
    int head; //pozycja kolejnego elementu do zapisania danych
    int tail; //pozycja ostanio odebranego elementu tablicy
    int maxlen;//maksymalna wielkość tablicy
    uint8_t bufforForBit;
    uint32_t numberAddBit;
    uint32_t time;
    DMA_HandleTypeDef *hdma_usart1_tx;
} uartCom_bbuf_t; // struktura bufora kołowego do wysyłania danych STM32->PC

uartCom_bbuf_t uartComBufforToCommunicationTx; //Bufor na dane które należy wysłać
uartCom_bbuf_t uartComBufforToCommunicationRx; //bufor na dane odebrane z konwertera
uartCom_bbuf_t uartComBufferToDiagnostics;
void uartComInit(uartCom_bbuf_t *buffer,DMA_HandleTypeDef *hdma);
void uartComPush(uartCom_bbuf_t *buffer,const uint8_t data);
void uartComAddBit(uartCom_bbuf_t *buffer,const uint8_t bit);
uint8_t *uartComGetBufferFirstElementAddress(uartCom_bbuf_t *buffer);
uint32_t uartComGetBufferLength(uartCom_bbuf_t *buffer);
uint16_t uartComPop(uartCom_bbuf_t *buffer);
void uartComResetByte(uartCom_bbuf_t *buffer);
void uartComTime();
uint32_t uartComGetSizeDataInBuffor(uartCom_bbuf_t *buffer);

#endif /* PERIPHERY_INC_UARTCOM_H_ */
