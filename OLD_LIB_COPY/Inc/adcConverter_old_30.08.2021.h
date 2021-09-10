/*
 * adcConverter.h
 *
 *  Created on: 10 sie 2021
 *      Author: DanielD
 */

#ifndef PERIPHERY_INC_ADCCONVERTER_OLD_30_08_2021_H_
#define PERIPHERY_INC_ADCCONVERTER_OLD_30_08_2021_H_
#define ADCCONVERTER_MAX_BUFFOR_LENGTH 134
#define INVALID_VALUE 1
#define ADCCONVERTER_FT 314900	//Czestotliwość próbkowania . Jednoska Hz
#define ADCCONVERTER_F1 4700 //Czestotliwość sinusa dla wartości binarnej 0. Jednoska Hz
#define ADCCONVERTER_F2 6700 //Czesotliwość sinusa dla wartości binarnej 1. Jednostka Hz

typedef enum{
	BUFFER_BUSY,
	BUFFER_FULL,
	BUFFER_CLEAR,
}Buffer_Status;

typedef struct{
	ADC_HandleTypeDef *hadc;
	DMA_HandleTypeDef *hdma_adc;
	uint16_t buffer1[ADCCONVERTER_MAX_BUFFOR_LENGTH];
	uint16_t buffer2[ADCCONVERTER_MAX_BUFFOR_LENGTH];
	uint16_t buffer3[ADCCONVERTER_MAX_BUFFOR_LENGTH];
	Buffer_Status buffer1Status;
	Buffer_Status buffer2Status;
	Buffer_Status buffer3Status;
	uint32_t targetFrequencyFor0;
	uint32_t targetFrequencyFor1;
	float scalingFactor;
	float omegaFor0;
	float omegaFor1;
	float sineFor0;
	float sineFor1;
	float cosineFor0;
	float cosineFor1;
	float coeffFor0;
	float coeffFor1;
}adcConverter_Primary_Struct_t;

adcConverter_Primary_Struct_t adcConverterPrimaryStruct;

void adcConverterInit(ADC_HandleTypeDef *hadc,DMA_HandleTypeDef *hdma_adc);
void adcConvert();
void adcEndConvert();
uint32_t adcGetConvertBit();
uint32_t adcConverterGoertzel(uint16_t* data,uint32_t numSamples);

#endif /* PERIPHERY_INC_ADCCONVERTER_OLD_30_08_2021_H_ */
