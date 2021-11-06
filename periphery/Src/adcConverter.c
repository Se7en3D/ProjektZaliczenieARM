/*
 * adcConverter.c
 *
 *  Created on: 30 sie 2021
 *      Author: DanielD
 */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "adcConverter.h"
#include "statemachine.h"


void adcConverterInit(ADC_HandleTypeDef *hadc,DMA_HandleTypeDef *hdma_adc){
	adcConverterPrimaryStruct.hadc=hadc;
	adcConverterPrimaryStruct.hdma_adc=hdma_adc;
	for(int i=0; i<ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR;i++){
		adcConverterPrimaryStruct.waveBufferStatus[i]=BUFFER_CLEAR;
	}
	//Poszukiwanie częstotliwości zbliżonej do oczekiwanej .Ogranicznie związane z algorytmem Goertzel-a
    int primaryFrequency=ADCCONVERTER_FT/ADCCONVERTER_MAX_BUFFOR_LENGTH; //czestotliwość początkowa odpowiadająca jednej próbce sygnału
	int frequency0=ADCCONVERTER_FT;
	int frequency1=ADCCONVERTER_FT;
	for(int i=primaryFrequency;i<ADCCONVERTER_FT;i+=primaryFrequency){
	    if(abs(i-ADCCONVERTER_F1)<abs(frequency0-ADCCONVERTER_F1)){
	        frequency0=i;
	    }
	    if(abs(i-ADCCONVERTER_F2)<abs(frequency1-ADCCONVERTER_F2)){
	        frequency1=i;
	    }
	}
	adcConverterPrimaryStruct.targetFrequencyFor0=frequency0;
	adcConverterPrimaryStruct.targetFrequencyFor1=frequency1;
	//Obliczenie zmiennych pomocniczych do liczenia algorytmu Goertzel-a
	int k;
	float floatnumSamples;
	adcConverterPrimaryStruct.scalingFactor = ADCCONVERTER_MAX_BUFFOR_LENGTH / 2.0;
    floatnumSamples = (float) ADCCONVERTER_MAX_BUFFOR_LENGTH;
    	//Czestotliwość dla zera
    k = (int) (0.5 + ((floatnumSamples * adcConverterPrimaryStruct.targetFrequencyFor0) / ADCCONVERTER_FT));
    adcConverterPrimaryStruct.omegaFor0 = (2.0 * M_PI * k) / floatnumSamples;
    adcConverterPrimaryStruct.sineFor0 = sin(adcConverterPrimaryStruct.omegaFor0);
    adcConverterPrimaryStruct.cosineFor0 = cos(adcConverterPrimaryStruct.omegaFor0);
    adcConverterPrimaryStruct.coeffFor0 = 2.0 * adcConverterPrimaryStruct.cosineFor0 ;

    	//Częstotliwość dla jedynki
    k = (int) (0.5 + ((floatnumSamples * adcConverterPrimaryStruct.targetFrequencyFor1) / ADCCONVERTER_FT));
    adcConverterPrimaryStruct.omegaFor1 = (2.0 * M_PI * k) / floatnumSamples;
    adcConverterPrimaryStruct.sineFor1 = sin(adcConverterPrimaryStruct.omegaFor1);
    adcConverterPrimaryStruct.cosineFor1 = cos(adcConverterPrimaryStruct.omegaFor1);
    adcConverterPrimaryStruct.coeffFor1 = 2.0 * adcConverterPrimaryStruct.cosineFor1;
}

void adcCoverterCheckDMAConversion(){
	if(adcConverterPrimaryStruct.status==ADCCONVERTER_WAIT){
			uint16_t sample=HAL_ADC_GetValue(adcConverterPrimaryStruct.hadc);
			if(sample>ADCCONVERTER_MIN_VALUE_SAMPLE_TO_START_DMA_CONVERSION){
				HAL_ADC_Stop_IT(adcConverterPrimaryStruct.hadc);

				adcConverterPrimaryStruct.status=ADCCONVERTERT_BUSY;
			}
	}

}

void adcConverterStartITConversion(){
	if(adcConverterPrimaryStruct.status==ADCCONVERTERT_IDLE){
		if(stateMachineStruct.status==stateMachineRecevingEnum){
			HAL_ADC_Start_IT(adcConverterPrimaryStruct.hadc);
			adcConverterPrimaryStruct.status=ADCCONVERTER_WAIT;
		}
	}
}

void adcConvert(){
	if(adcConverterPrimaryStruct.hdma_adc->State!=HAL_DMA_STATE_READY){ //Sprawdzenie czy DMA jest gotowy do uruchomienia
		return;
	}
	if(adcConverterPrimaryStruct.status==ADCCONVERTERT_BUSY){
		adcEndConvert();
		uint32_t next=adcConverterPrimaryStruct.head+1;
		if(next>=ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR){
			next=0;
		}
		if(next==adcConverterPrimaryStruct.tail){
			return;
		}

		if(adcConverterPrimaryStruct.waveBufferStatus[next]==BUFFER_CLEAR){
			adcConverterPrimaryStruct.status=ADCCONVERTERT_BUSY;
			HAL_ADC_Start_DMA(adcConverterPrimaryStruct.hadc,(uint32_t*) adcConverterPrimaryStruct.waveBuffer[next], ADCCONVERTER_MAX_BUFFOR_LENGTH);
			adcConverterPrimaryStruct.waveBufferStatus[next]=BUFFER_BUSY;
			adcConverterPrimaryStruct.head=next;
		}
	}
}

void adcEndConvert(){
	if(adcConverterPrimaryStruct.waveBufferStatus[adcConverterPrimaryStruct.head]==BUFFER_BUSY){
		adcConverterPrimaryStruct.waveBufferStatus[adcConverterPrimaryStruct.head]=BUFFER_FULL;
		adcConverterPrimaryStruct.status=ADCCONVERTERT_IDLE;
		adcConverterPrimaryStruct.countConvertData++;
	}
}

uint32_t adcGetConvertBit(){
	uint32_t convertBit=ADCCONVERTER_ALL_STATUS_NO_FULL;
	uint32_t next=adcConverterPrimaryStruct.tail;


	switch(adcConverterPrimaryStruct.waveBufferStatus[next]){
		case BUFFER_CLEAR:
			break;
		case BUFFER_BUSY:
			return convertBit;
			break;
		case BUFFER_FULL:
			convertBit=adcConverterGoertzel(adcConverterPrimaryStruct.waveBuffer[next],ADCCONVERTER_MAX_BUFFOR_LENGTH);
			adcConverterPrimaryStruct.waveBufferStatus[next]=BUFFER_CLEAR;
			break;
		default:
			break;
	}
	if(next==adcConverterPrimaryStruct.head){
		return convertBit;
	}else{
		next++;
		if(next>ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR){
			next=0;
		}
		adcConverterPrimaryStruct.tail=next;
		return convertBit;
	}
}

uint32_t adcConverterGoertzel(uint16_t* data,uint32_t numSamples){
	float   q0For0=0,q1For0=0,q2For0=0,realFor0,imagFor0,magnitudeFor0;
	float   q0For1=0,q1For1=0,q2For1=0,realFor1,imagFor1,magnitudeFor1;
		//Obliczenie współczynników dla q0,q1,q2
    for(int i=0; i<numSamples; i++)
    {
    	q0For0 = adcConverterPrimaryStruct.coeffFor0 * q1For0 - q2For0 + data[i];
    	q2For0 = q1For0;
    	q1For0 = q0For0;


    	q0For1 = adcConverterPrimaryStruct.coeffFor1 * q1For1 - q2For1 + data[i];
    	q2For1 = q1For1;
    	q1For1 = q0For1;
    }

    	//Obliczenie mocy dla konkrentych częstotliwości
    realFor0 = (q1For0 - q2For0 * adcConverterPrimaryStruct.cosineFor0) / adcConverterPrimaryStruct.scalingFactor;
    imagFor0 = (q2For0 * adcConverterPrimaryStruct.sineFor0) / adcConverterPrimaryStruct.scalingFactor;

    magnitudeFor0 = sqrtf(realFor0*realFor0 + imagFor0*imagFor0);

    realFor1 = (q1For1 - q2For1 * adcConverterPrimaryStruct.cosineFor1) / adcConverterPrimaryStruct.scalingFactor;
    imagFor1 = (q2For1 * adcConverterPrimaryStruct.sineFor1) / adcConverterPrimaryStruct.scalingFactor;

    magnitudeFor1 = sqrtf(realFor1*realFor1 + imagFor1*imagFor1);

    if(magnitudeFor1>ADCCONVERTER_MIN_VALUE_GOERTZEL ||magnitudeFor0>ADCCONVERTER_MIN_VALUE_GOERTZEL){
    	stateMachineResetTime(); //Reset czasu braku transmisji;
    	adcConverterPrimaryStruct.countNoneTransmission=0;
		if(magnitudeFor0>magnitudeFor1){
			return 0;
		}else{
			return 1;
		}}

    adcConverterPrimaryStruct.status=ADCCONVERTERT_IDLE;
    if(adcConverterPrimaryStruct.countNoneTransmission>ADCCONVERTER_AMOUND_OF_DATA_NEEDED_FOR_THE_RESET_UART_BYTE){
    	adcConverterPrimaryStruct.countNoneTransmission=0;
    	return ADCCONVERTER_RESET_UART_BYTE;
    }
    return ADCCONVERTER_ALL_STATUS_NO_FULL;
}
void adcConverterStop(){
	switch(adcConverterPrimaryStruct.status){
	case ADCCONVERTERT_IDLE:
		break;
	case ADCCONVERTER_WAIT:
		adcConverterPrimaryStruct.status=ADCCONVERTERT_IDLE;
		break;
	case ADCCONVERTERT_BUSY:
		if(adcConverterPrimaryStruct.hdma_adc->State==HAL_DMA_STATE_BUSY){
			HAL_ADC_Stop_DMA(adcConverterPrimaryStruct.hadc);
			adcConverterPrimaryStruct.status=ADCCONVERTERT_IDLE;
			int head=adcConverterPrimaryStruct.head;
			if(adcConverterPrimaryStruct.waveBufferStatus[head]==BUFFER_BUSY){
				adcConverterPrimaryStruct.waveBufferStatus[head]=BUFFER_CLEAR;
				if(head==0){
					adcConverterPrimaryStruct.head=ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR-1;
				}else{
					adcConverterPrimaryStruct.head--;
				}

			}
		}
		break;
	}
}


