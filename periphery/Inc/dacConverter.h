/*
 * dacConverter.h
 *
 *  Created on: 10 sie 2021
 *      Author: DanielD
 */

/*
 * dacConverter.h
 *
 *  Created on: 2 sie 2021
 *      Author: DanielD
 */
#define DACCINVERTER_MAX_BIT_TO_SEND 7 //Bity przesyłane podczas jednej sekwencji
#define DACCONVERTER_TIME_BREAK 2 //Czas trwania przerwy w nadania (ms)
#define DACCONVERTER_MAX_BYTE_TO_SEND_WITHOUT_BREAK 4 //ilość wysłanych bajtów bez przerwy
typedef enum{
	DACCONVERTER_IDLE=0,
	DACCONVERTER_BUSY,
	DACCONVERTER_BREAK
} dacConverterStatus;

typedef struct{
	DAC_HandleTypeDef *hdac;
	DMA_HandleTypeDef *hdma_dac1;
	uint8_t bitSendNumber;
	uint8_t dataToSend;
	uint32_t numberOfContinousConversions;
	uint32_t timeBreak;
	dacConverterStatus status;
	uint32_t sizeWaveF0;
	uint32_t sizeWaveF1;
} dac_Base_Struct;

dac_Base_Struct dacBaseStruct;

void dacConverterInit(DAC_HandleTypeDef *hdac);
void dacConverterSetNewDataToSend(uint8_t data);
void dacNextBitConvert();
void dacAddTimeBraek();


