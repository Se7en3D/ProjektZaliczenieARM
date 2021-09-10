/*
 * stateMachine.h
 *
 *  Created on: 9 wrz 2021
 *      Author: DanielD
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

typedef enum{
	stateMachineIdleEnum,
	stateMachineSendingEnum,
	stateMachineRecevingEnum,
}stateMachineHandleMainEnum;

typedef struct{
	stateMachineHandleMainEnum status;
	uint32_t time;
}stateMachineHandleMainStruct;


void stateMachineInit();
void stateMachineIdle();
void stateMachineSending();
void stateMachineReceving();
void stateMachineAddTime();

#endif /* INC_STATEMACHINE_H_ */
