/**
  ******************************************************************************
  * @file    dacConverter.h
  * @author  Daniel Dunak
  * @brief   Plik nagłówkowy odpowiedzialny za kontrolę nad przetwornikiem konwertującym bity na przebieg sinusoidalny
  * 			wyjście PA4
  *
  ******************************************************************************
  */
#define DACCINVERTER_MAX_BIT_TO_SEND 7 /*!<Bity przesyłane podczas jednej sekwencji*/
#define DACCONVERTER_TIME_BREAK 2 /*!<Czas trwania przerwy w nadania (ms)*/
#define DACCONVERTER_MAX_BYTE_TO_SEND_WITHOUT_BREAK 4 /*!<ilość wysłanych bajtów bez przerwy*/

/**
 * @brief dacConverterStatus status konwertera DAC
 */
typedef enum{
	DACCONVERTER_IDLE=0,/*!<Oczekiwanie na bajty do wysłania */
	DACCONVERTER_BUSY,/*!<Wysyłanie bitów*/
	DACCONVERTER_BREAK/*!<Przerwa w konwersji występuje po wysłaniu DACCONVERTER_MAX_BYTE_TO_SEND_WITHOUT_BREAK bitów*/
} dacConverterStatus;

/**
 * @brief dac_Base_Struct główna struktura przetwornika DAC
 */
typedef struct{
	DAC_HandleTypeDef *hdac;/*!<Wskaźnik do DAC_HandleTypeDef*/
	DMA_HandleTypeDef *hdma_dac1;/*!<Wskażnik do DMA_HandleTypeDef*/
	uint8_t bitSendNumber;/*!<Numer obecnie wysyłanego bitu*/
	uint8_t dataToSend;/*!<Bajt który jest wysyłany*/
	uint32_t numberOfContinousConversions;/*!<Ilosc bez przerwowej konwersji*/
	uint32_t timeBreak;/*!<Czas odliczający koniec przerwy*/
	dacConverterStatus status;/*!<Status przetwornika*/
	uint32_t sizeWaveF0;/*!<Wielkość bufora wypełnienia sinusoidy dla bitu 0*/
	uint32_t sizeWaveF1;/*!<Wielkość bufora wypełnienia sinusoidy dla bitu 1*/
} dac_Base_Struct;

dac_Base_Struct dacBaseStruct; /*!<Referencja do struktury dac_Base_Struct*/

void dacConverterInit(DAC_HandleTypeDef *hdac);
void dacConverterSetNewDataToSend(uint8_t data);
void dacNextBitConvert();
void dacAddTimeBraek();


