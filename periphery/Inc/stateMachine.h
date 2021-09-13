/*
 * stateMachine.h
 *
 *  Created on: 9 wrz 2021
 *      Author: DanielD
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#define STATEMACHINE_TIME_UNTIL_THE_STATE_CHANGES_TO_SENDING 100 //Czas po którym może nastapić zmiana stanu z stateMachineRecevingEnum na stateMachineSendingEnum
#define STATEMACHINE_RETURN_TRUE 1;
#define STATEMACHINE_RETURN_FALSE 0;
typedef enum{
	stateMachineRecevingEnum,
	stateMachineSendingEnum,
}stateMachineHandleMainEnum;

typedef struct{
	stateMachineHandleMainEnum status;
	uint32_t time;
}stateMachineHandleMainStruct;

stateMachineHandleMainStruct stateMachineStruct;
void stateMachineInit();
void stateMachineAddTime();
void stateMachineResetTime();
uint16_t stateMachineChangeToReceving();
uint16_t stateMachineChangeToSending();

#endif /* INC_STATEMACHINE_H_ */
