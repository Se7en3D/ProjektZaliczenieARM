/**
  ******************************************************************************
  * @file    stateMachine.h
  * @author  Daniel Dunak
  * @brief   Plik nagłówkowy odpowiedzialny funkcje obsługi
  * 			Mikrokontroler konwertuje dane odpierane przez uart na sinusoidy o czestotliwości 4700 Hz dla
  * 			logicznego zera a 6700Hz dla logicznej jedynki. Generacją sinusoidy zajmuję sie przetwornika cyfrowo analogowy. Sinusoidy odbierane są przez przetwornika
  * 			analogowo cyfrowy. Po odebraniu opowiedniej ilości danych mikrokontroler poprzez algorytm Gortzela sprawdza moc dla konkretnej częstotliwości i na tej
  * 			podstawie ustala czy odebrana wartośc do logiczne 0 lub 1. Jesli odebrane zostanie 8 bitów wartość bajtowa zostanie przesłana na magistralę uart.
  *
  ******************************************************************************
  */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#define STATEMACHINE_TIME_UNTIL_THE_STATE_CHANGES_TO_SENDING 100 /*!<Czas po którym może nastapić zmiana stanu z stateMachineRecevingEnum na stateMachineSendingEnum*/
#define STATEMACHINE_RETURN_TRUE 1;/*!<Wartość zwracająca jeśli zmieniono stan nadajnika<->odbiornika*/
#define STATEMACHINE_RETURN_FALSE 0; /*!<Wartość zwracająca jeśni nie zniemiono stanu*/
/**
 * @brief stateMachineHandleMainEnum typ wyliczeniowy maszyny stanów
 */
typedef enum{
	stateMachineRecevingEnum,/*!<Układ pracuje jako odbiornik danych*/
	stateMachineSendingEnum,/*!<Układ pracuje jako nadajnik danych*/
}stateMachineHandleMainEnum;
/**
 * @brief Główna struktura maszyny stanów
 */
typedef struct{
	stateMachineHandleMainEnum status;/*!<Aktualny stan maszyny*/
	uint32_t time;/*!<zmianna czas*/
}stateMachineHandleMainStruct;

stateMachineHandleMainStruct stateMachineStruct; /*!<Referencja do struktury stateMachineHandleMainStruct*/


void stateMachineAddTime();
void stateMachineResetTime();
uint16_t stateMachineChangeToReceving();
uint16_t stateMachineChangeToSending();

#endif /* INC_STATEMACHINE_H_ */
