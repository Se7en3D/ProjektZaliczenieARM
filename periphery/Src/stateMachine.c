/**
  ******************************************************************************
  * @file    stateMachine.c
  * @author  Daniel Dunak
  * @brief   Plik zródłowy odpowiedzialny funkcje obsługi
  *
  ******************************************************************************
  */
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stateMachine.h"
#include "adcConverter.h"

/**
  * @brief	Zwiekszenie zmiennej czasowej
  * @retval None
  */
void stateMachineAddTime(){
	if(stateMachineStruct.status==stateMachineRecevingEnum){
		stateMachineStruct.time++;
	}else{
		stateMachineStruct.time=0;
	}
}

/**
  * @brief	Reset zmiennej czasowej
  * @retval None
  */
void stateMachineResetTime(){
	stateMachineStruct.time=0;
}

/**
  * @brief	Zmiana stanu wysyłania do odbierania
  * @retval uint16_t STATEMACHINE_RETURN_TRUE gdy stan został zmieniony lub STATEMACHINE_RETURN_FALSE gdy stan nie został zmieniony
  */
uint16_t stateMachineChangeToReceving(){
	if(stateMachineStruct.status==stateMachineSendingEnum){
		stateMachineStruct.status=stateMachineRecevingEnum;
		return STATEMACHINE_RETURN_TRUE;
	}else{
		return STATEMACHINE_RETURN_FALSE;
	}
}

/**
  * @brief	Zmiana stanu odbierania do wysyłania
  * @retval uint16_t STATEMACHINE_RETURN_TRUE gdy stan został zmieniony lub STATEMACHINE_RETURN_FALSE gdy stan nie został zmieniony
  */
uint16_t stateMachineChangeToSending(){
	if(stateMachineStruct.status==stateMachineRecevingEnum && stateMachineStruct.time>=STATEMACHINE_TIME_UNTIL_THE_STATE_CHANGES_TO_SENDING){
		stateMachineStruct.status=stateMachineSendingEnum;
		adcConverterStop();
		return STATEMACHINE_RETURN_TRUE;
	}else{
		return STATEMACHINE_RETURN_FALSE;
	}

}
