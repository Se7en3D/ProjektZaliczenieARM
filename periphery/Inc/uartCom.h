/**
  ******************************************************************************
  * @file    uartCom.h
  * @author  Daniel Dunak
  * @brief   Plik nagłówkowy komunikacji po portcie UART
  * 			nadajnik(TX) na wyjściu PC12 a odbiornik(RX) na wejściu PD2
  *
  ******************************************************************************
  */

#ifndef PERIPHERY_INC_UARTCOM_H_
#define PERIPHERY_INC_UARTCOM_H_
#include <stdio.h>
#include "stm32f4xx_hal.h"
#define UARTCOM_BUF_MAXLEN 100 /*!<Maksymalna wielkość bufora nadajnika STM32->PC*/
#define UARTCOM_NO_DATA_TO_POP 300 /*!<Brak danych do pobrania*/
#define UARTCOM_TIME_TO_DELETE_BUFFOR_FOR_DATA 10 /*!<zas po którym należy wysłać/usunąć znajdujące się bity jeśli nie odebrano żadnych informacji*/


/**
  * @brief  Struktura bufora kołowego oraz bufora od dekodowania bitów transmisji
  * */
typedef struct {
    uint8_t buffer[UARTCOM_BUF_MAXLEN]; /*!< Bufor próbek o wielkości 8 bitów jego wielkość jest zalezna od parametru UARTCOM_BUF_MAXLEN*/
    uint32_t head; /*!<ozycja kolejnego elementu do zapisania danych*/
    uint32_t tail; /*!<pozycja ostanio odebranego elementu tablicy*/
    uint32_t maxlen;/*!<maksymalna wielkość tablicy*/
    uint8_t bufforForBit;/*!<Bufor do składania z bitów bajtów */
    uint32_t numberAddBit;/*!<Ilość dodanych bitów do zmiennej bufforForBit*/
    uint32_t time;/*!<Zmienna określająca czas*/
    DMA_HandleTypeDef *hdma_usart1_tx;/*!< wskażnik do struktury DMA_HandleTypeDef*/
} uartCom_bbuf_t;

uartCom_bbuf_t uartComBufforToCommunicationTx; /*!<struktura buforu na dane które należy wysłać*/
uartCom_bbuf_t uartComBufforToCommunicationRx; /*!<struktura buforu na dane odebrane z konwertera*/
uartCom_bbuf_t uartComBufferToDiagnostics;/*!<struktura na bufor na dane diagnostyczne*/
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
