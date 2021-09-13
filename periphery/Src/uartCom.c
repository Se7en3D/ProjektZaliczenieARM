/*
 * uartCom.c
 *
 *  Created on: 13 sie 2021
 *      Author: DanielD
 */
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "uartCom.h"

/**
  * @brief  zainicjowanie struktury uartCom_bbuf_t.
  * @param  buffer wzkażnik do struktury uartCom_bbuf_t która ma zostac zainicjalizowana oraz do której
  * 		mają zostać wpisane podstawowe zmienne
  * @param  hdma wskażnik do struktury DMA_HandleTypeDef potrzebnej do automatycznego wysyłania danych
  * @retval brak
  */
void uartComInit(uartCom_bbuf_t *buffer,DMA_HandleTypeDef *hdma){
	if(hdma!=0)
		buffer->hdma_usart1_tx=hdma;

	buffer->maxlen=UARTCOM_BUF_MAXLEN;
}


void uartComPush(uartCom_bbuf_t *buffer,const uint8_t data){
	if(buffer->maxlen!=UARTCOM_BUF_MAXLEN){
		return;
	}
	int next;
	next = buffer->head + 1;  // next is where head will point to after this write.
	if (next >= (buffer->maxlen))
			next = 0;

	if (next == buffer->tail){  // if the head + 1 == tail, circular buffer is full
		buffer->buffer[buffer->head] = data;  // Load data and then move
		return ;
	}

	buffer->buffer[buffer->head] = data;  // Load data and then move
	buffer->head = next;             // head to next data offset.
}

void uartComAddBit(uartCom_bbuf_t *buffer,const uint8_t bit){

	buffer->bufforForBit|=(bit&0x01)<<buffer->numberAddBit;
	buffer->numberAddBit++;
	buffer->time=0;
	if(buffer->numberAddBit>7){
		uartComPush(buffer,buffer->bufforForBit);
		buffer->bufforForBit=0;
		buffer->numberAddBit=0;
	}
}
uint8_t *uartComGetBufferFirstElementAddress(uartCom_bbuf_t *buffer){
	if(buffer->time>UARTCOM_TIME_TO_DELETE_BUFFOR_FOR_DATA){
		if(buffer->numberAddBit>0){
			buffer->time=0;
			//uartComPush(buffer,buffer->bufforForBit);
			buffer->bufforForBit=0;
			buffer->numberAddBit=0;
		}
	}
	return &buffer->buffer[buffer->tail];
}

uint32_t uartComGetBufferLength(uartCom_bbuf_t *buffer){
	if(buffer->head==buffer->tail){
		return 0;
	}

	if(buffer->head<buffer->tail){
		uint32_t length=buffer->maxlen-buffer->tail;
		buffer->tail=0;
		return length;
	}else{
		uint32_t length=buffer->head-buffer->tail;
		buffer->tail=buffer->head;
		return length;
	}
}

uint16_t uartComPop(uartCom_bbuf_t *buffer){
	uint16_t data;
	int next;
	if(buffer->maxlen!=UARTCOM_BUF_MAXLEN){
		return UARTCOM_NO_DATA_TO_POP;
	}

	if (buffer->head == buffer->tail){  // if the head == tail, we don't have any data
		return UARTCOM_NO_DATA_TO_POP;
	}
	next = buffer->tail + 1;  // next is where tail will point to after this read.
	if (next >= buffer->maxlen)
		next = 0;

	data = buffer->buffer[buffer->tail];  // Read data and then move
	buffer->tail = next;              // tail to next offset.
	return data;  // return success to indicate successful push.
}
void uartComResetByte(uartCom_bbuf_t *buffer){
	buffer->numberAddBit=0;
	buffer->bufforForBit=0;
}

void uartComTime(){
	if(uartComBufforToCommunicationTx.numberAddBit>0){
		uartComBufforToCommunicationTx.time++;
	}
}
uint32_t uartComGetSizeDataInBuffor(uartCom_bbuf_t *buffer){
	if(buffer->tail==buffer->head){
		return 0;
	}
	if(buffer->head>buffer->tail){
		return buffer->head-buffer->tail;
	}else{
		return buffer->maxlen-buffer->tail+buffer->head;
	}

}
