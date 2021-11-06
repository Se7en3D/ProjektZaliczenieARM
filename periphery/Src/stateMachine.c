/*
 * stateMachine.c
 *
 *  Created on: 9 wrz 2021
 *      Author: DanielD
 */
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stateMachine.h"
#include "adcConverter.h"


void stateMachineAddTime(){
	if(stateMachineStruct.status==stateMachineRecevingEnum){
		stateMachineStruct.time++;
	}else{
		stateMachineStruct.time=0;
	}
}

void stateMachineResetTime(){
	stateMachineStruct.time=0;
}

uint16_t stateMachineChangeToReceving(){
	if(stateMachineStruct.status==stateMachineSendingEnum){
		stateMachineStruct.status=stateMachineRecevingEnum;
		return STATEMACHINE_RETURN_TRUE;
	}else{
		return STATEMACHINE_RETURN_FALSE;
	}
}
uint16_t stateMachineChangeToSending(){
	if(stateMachineStruct.status==stateMachineRecevingEnum && stateMachineStruct.time>=STATEMACHINE_TIME_UNTIL_THE_STATE_CHANGES_TO_SENDING){
		stateMachineStruct.status=stateMachineSendingEnum;
		adcConverterStop();
		return STATEMACHINE_RETURN_TRUE;
	}else{
		return STATEMACHINE_RETURN_FALSE;
	}

}
