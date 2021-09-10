/*
 * adcConverter.c
 *
 *  Created on: 10 sie 2021
 *      Author: DanielD
 */
#include <adcConverter_old_30.08.2021.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void adcConverterInit(ADC_HandleTypeDef *hadc,DMA_HandleTypeDef *hdma_adc){
	adcConverterPrimaryStruct.hadc=hadc;
	adcConverterPrimaryStruct.hdma_adc=hdma_adc;
	adcConverterPrimaryStruct.buffer1Status=BUFFER_CLEAR;
	adcConverterPrimaryStruct.buffer2Status=BUFFER_CLEAR;
	adcConverterPrimaryStruct.buffer3Status=BUFFER_CLEAR;
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
void adcConvert(){
	if(adcConverterPrimaryStruct.hdma_adc->State!=HAL_DMA_STATE_READY){ //Sprawdzenie czy DMA jest gotowy do uruchomienia
		return;
	}
	if(adcConverterPrimaryStruct.buffer1Status==BUFFER_CLEAR){
		HAL_ADC_Start_DMA(adcConverterPrimaryStruct.hadc,(uint32_t*) adcConverterPrimaryStruct.buffer1, ADCCONVERTER_MAX_BUFFOR_LENGTH);
		adcConverterPrimaryStruct.buffer1Status=BUFFER_BUSY;
		adcEndConvert();
		return;
	}
	if(adcConverterPrimaryStruct.buffer1Status==BUFFER_CLEAR){
		HAL_ADC_Start_DMA(adcConverterPrimaryStruct.hadc,(uint32_t*) adcConverterPrimaryStruct.buffer2, ADCCONVERTER_MAX_BUFFOR_LENGTH);
		adcConverterPrimaryStruct.buffer2Status=BUFFER_BUSY;
		adcEndConvert();
			return;
	}
	if(adcConverterPrimaryStruct.buffer1Status==BUFFER_CLEAR){
		HAL_ADC_Start_DMA(adcConverterPrimaryStruct.hadc,(uint32_t*) adcConverterPrimaryStruct.buffer3, ADCCONVERTER_MAX_BUFFOR_LENGTH);
		adcConverterPrimaryStruct.buffer3Status=BUFFER_BUSY;
		adcEndConvert();
			return;
	}
	//Error Wszystkie bufory są zajęte
}


void adcEndConvert(){
	// Sprawdzenie czy flaga oznaczająca zajętość buforów znajduje się w większej ilości slotów niż 1
	int busyFlag=0;

	if(adcConverterPrimaryStruct.buffer1Status==BUFFER_BUSY){
		busyFlag++;
		adcConverterPrimaryStruct.buffer1Status=BUFFER_FULL;
	}
	if(adcConverterPrimaryStruct.buffer2Status==BUFFER_BUSY){
		busyFlag++;
		adcConverterPrimaryStruct.buffer2Status=BUFFER_FULL;
	}
	if(adcConverterPrimaryStruct.buffer3Status==BUFFER_BUSY){
		busyFlag++;
		adcConverterPrimaryStruct.buffer3Status=BUFFER_FULL;
	}
	if(busyFlag>1){ //bład związany z większą ilością flag niż powinno być
		busyFlag=0;
	}
}

uint32_t adcGetConvertBit(){
	uint32_t convertBit=2;
	if(adcConverterPrimaryStruct.buffer1Status==BUFFER_FULL){
		convertBit=adcConverterGoertzel(adcConverterPrimaryStruct.buffer1,ADCCONVERTER_MAX_BUFFOR_LENGTH);
		adcConverterPrimaryStruct.buffer1Status=BUFFER_CLEAR;
		return convertBit;
	}
	if(adcConverterPrimaryStruct.buffer2Status==BUFFER_FULL){
		convertBit=adcConverterGoertzel(adcConverterPrimaryStruct.buffer2,ADCCONVERTER_MAX_BUFFOR_LENGTH);
		adcConverterPrimaryStruct.buffer2Status=BUFFER_CLEAR;
		return convertBit;
	}
	if(adcConverterPrimaryStruct.buffer3Status==BUFFER_FULL){
		convertBit=adcConverterGoertzel(adcConverterPrimaryStruct.buffer3,ADCCONVERTER_MAX_BUFFOR_LENGTH);
		adcConverterPrimaryStruct.buffer3Status=BUFFER_CLEAR;
		return convertBit;
	}

	return convertBit;
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


    if(magnitudeFor0>magnitudeFor1){
    	return 0;
    }else{
    	return 1;
    }
}

float adcConverterGoertzelBeta(int numSamples,int TARGET_FREQUENCY,int SAMPLING_RATE, int* data){
    int     k,i;
    float   floatnumSamples;
    float   omega,sine,cosine,coeff,q0,q1,q2,magnitude,real,imag;

    float   scalingFactor = numSamples / 2.0;

    floatnumSamples = (float) numSamples;
    k = (int) (0.5 + ((floatnumSamples * TARGET_FREQUENCY) / SAMPLING_RATE));
    omega = (2.0 * M_PI * k) / floatnumSamples;
    sine = sin(omega);
    cosine = cos(omega);
    coeff = 2.0 * cosine;
    q0=0;
    q1=0;
    q2=0;

    for(i=0; i<numSamples; i++)
    {
        q0 = coeff * q1 - q2 + data[i];
        q2 = q1;
        q1 = q0;
    }

    // calculate the real and imaginary results
    // scaling appropriately
    real = (q1 - q2 * cosine) / scalingFactor;
    imag = (q2 * sine) / scalingFactor;

    magnitude = sqrtf(real*real + imag*imag);
    return magnitude;
}
