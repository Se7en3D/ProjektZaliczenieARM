/*
 * adcConverter.h
 *
 *  Created on: 30 sie 2021
 *      Author: DanielD
 */

#ifndef PERIPHERY_INC_ADCCONVERTER_H_
#define PERIPHERY_INC_ADCCONVERTER_H_

#include "stm32f4xx_hal.h"

#define ADCCONVERTER_MAX_BUFFOR_LENGTH 134 //Wielkość bufora próbek sygnału do analizy
#define ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR 32 //Ilość buforów z próbkami sygnału
#define ADCCONVERTER_ALL_STATUS_NO_FULL 3 //Wartość informująca o
#define ADCCONVERTER_MIN_VALUE_GOERTZEL 500.0
#define ADCCONVERTER_AMOUND_OF_DATA_NEEDED_FOR_THE_RESET_UART_BYTE 3 // ilość danych potrzebna do resetu bufora odbiornika bitowego
#define ADCCONVERTER_RESET_UART_BYTE 4 //Wartość informująca o konieczności resetu bufora bajtowego
#define ADCCONVERTER_FT 314900	//Czestotliwość próbkowania . Jednoska Hz
#define ADCCONVERTER_F1 4700 //Czestotliwość sinusa dla wartości binarnej 0. Jednoska Hz
#define ADCCONVERTER_F2 6700 //Czesotliwość sinusa dla wartości binarnej 1. Jednostka Hz
#define ADCCONVERTER_MIN_VALUE_SAMPLE_TO_START_DMA_CONVERSION 2200 //minimalna wartość napięcia od której zaczynana jest konwersja DMA

typedef enum{
	BUFFER_BUSY,
	BUFFER_FULL,
	BUFFER_CLEAR,
}Buffer_Status;

typedef enum{
	ADCCONVERTERT_IDLE=0,
	ADCCONVERTER_WAIT,
	ADCCONVERTERT_BUSY,
} adcConverterStatus;

typedef struct{
	ADC_HandleTypeDef *hadc;
	DMA_HandleTypeDef *hdma_adc;
	uint16_t waveBuffer[ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR][ADCCONVERTER_MAX_BUFFOR_LENGTH];
	Buffer_Status waveBufferStatus[ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR];
	uint32_t targetFrequencyFor0;
	uint32_t targetFrequencyFor1;
	uint32_t lastBufferFull;
	uint32_t countNoneTransmission;
	uint32_t head;
	uint32_t tail;
	adcConverterStatus status;
	float scalingFactor;
	float omegaFor0;
	float omegaFor1;
	float sineFor0;
	float sineFor1;
	float cosineFor0;
	float cosineFor1;
	float coeffFor0;
	float coeffFor1;
	uint32_t countConvertData;
}adcConverter_Primary_Struct_t;

adcConverter_Primary_Struct_t adcConverterPrimaryStruct;

void adcConverterInit(ADC_HandleTypeDef *hadc,DMA_HandleTypeDef *hdma_adc);
void adcCoverterCheckDMAConversion();
void adcConverterStartITConversion();
void adcConvert();
void adcEndConvert();
uint32_t adcGetConvertBit();
uint32_t adcConverterGoertzel(uint16_t* data,uint32_t numSamples);
void adcConverterStop();

#endif /* PERIPHERY_INC_ADCCONVERTER_H_ */
