/**
  ******************************************************************************
  * @file    adcConverter.h
  * @author  Daniel Dunak
  * @brief   Plik zródłowy odpowiedzialny za kontrolę nad przetwornikiem pobierającym próbki sygnału
  * 			wejście PA1
  *
  ******************************************************************************
  */


#define ADCCONVERTER_MAX_BUFFOR_LENGTH 134 /*!<Wielkość bufora próbek sygnału do analizy*/
#define ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR 32 /*!<Ilość buforów z próbkami sygnału*/
#define ADCCONVERTER_ALL_STATUS_NO_FULL 3 /*!<Wartość informująca o braku kolejnego bitu*/
#define ADCCONVERTER_MIN_VALUE_GOERTZEL 500.0 /*!<Minimalna wartość obliczonego współczynnika mocy potrzebna do detekcji wartości binarnej*/
#define ADCCONVERTER_AMOUND_OF_DATA_NEEDED_FOR_THE_RESET_UART_BYTE 3 /*!<ilość danych potrzebna do resetu bufora odbiornika bitowego*/
#define ADCCONVERTER_RESET_UART_BYTE 4 /*!<Wartość informująca o konieczności resetu bufora bajtowego*/
#define ADCCONVERTER_FT 314900	/*!<Czestotliwość próbkowania . Jednoska Hz*/
#define ADCCONVERTER_F1 4700 /*!<Czestotliwość sinusa dla wartości binarnej 0. Jednoska Hz*/
#define ADCCONVERTER_F2 6700 /*!<Czesotliwość sinusa dla wartości binarnej 1. Jednostka Hz*/
#define ADCCONVERTER_MIN_VALUE_SAMPLE_TO_START_DMA_CONVERSION 2200 /*!<minimalna wartość napięcia od której zaczynana jest konwersja DMA*/

/**
  * @brief  Buffer_Status status tablicy zawierającej próbki danych
  */
typedef enum{
	BUFFER_BUSY,/*!< Tablica jest obecnie wypełniana danymi*/
	BUFFER_FULL,/*!< Tablica została wypełniona danymi*/
	BUFFER_CLEAR,/*!< Tablica jest pusta*/
}Buffer_Status;

/**
  * @brief  adcConverterStatus odpowiada za informację na temat obecnego statusu prztwornika ADC
  */
typedef enum{
	ADCCONVERTERT_IDLE=0,/*!< Przetwornik jest w trybie bezczynności*/
	ADCCONVERTER_WAIT,/*!< Przetwornik w trybie ciągłym sprawdza czy napięcie jest większe od stałej ADCCONVERTER_MIN_VALUE_SAMPLE_TO_START_DMA_CONVERSION */
	ADCCONVERTERT_BUSY,/*!< Prztwornik zapisuje próbki do tablic w celu obliczenia przesyłanych danych*/
} adcConverterStatus;

/**
  * @brief adcConverter_Primary_Struct_t glowna struktura przetwornika ADC
  */
typedef struct{
	ADC_HandleTypeDef *hadc; /*!< uchwyt do ADC_HandleTypeDef*/
	DMA_HandleTypeDef *hdma_adc;/*!<uchwyt do DMA_HandleTypeDef*/
	uint16_t waveBuffer[ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR][ADCCONVERTER_MAX_BUFFOR_LENGTH];/*!<Tablica zawierająca probki sygnalu*/
	Buffer_Status waveBufferStatus[ADCCONVERTER_MAX_LENGTH_ARRAY_OF_BUFFOR];/*!<Tablica zawierająca statusy poszczególnych tablic zawierających próbki sygnalu*/
	uint32_t targetFrequencyFor0;/*!<Czestoliwosc dla 0*/
	uint32_t targetFrequencyFor1;/*!<Czestotliwosc dla 1*/
	uint32_t lastBufferFull;/*!<Numer ostatniego bufora ktory byl pelny*/
	uint32_t countNoneTransmission;/*!<Zlicza ilosc bitow ktore nie zostaly rozpoznane*/
	uint32_t head;/*!< wartosc numeryczna potrzebna do detekcji ktora tablice nalezy uzupełnic danymi*/
	uint32_t tail;/*!<wartosc numeryczna informujaca o obecnie sprawdzanym buforze danych*/
	adcConverterStatus status;/*!<status przetwornika ADC*/
	float scalingFactor;/*!<scalingFactor */
	float omegaFor0;/*!< omega dla bitu o wartosci 0*/
	float omegaFor1;/*!<omega dla bitu o wartosci 1*/
	float sineFor0;/*!< sin dla bitu o wartosci 0*/
	float sineFor1;/*!< sin dla bitu o wartosci 1*/
	float cosineFor0;/*!< cos dla bitu o wartosci 0*/
	float cosineFor1;/*!<cos dla bitu o wartosci 1*/
	float coeffFor0;/*!<coeff dla bitu o wartosci 0*/
	float coeffFor1;/*!<coeff fla bitu o wartosci 1*/
	uint32_t countConvertData;/*!<zlicza ilosc bitow ktore zostaly przekonwertowane*/
}adcConverter_Primary_Struct_t;

adcConverter_Primary_Struct_t adcConverterPrimaryStruct; /*!<Referencja do struktury adcConverter_Primary_Struct_t*/

void adcConverterInit(ADC_HandleTypeDef *hadc,DMA_HandleTypeDef *hdma_adc);
void adcCoverterCheckDMAConversion();
void adcConverterStartITConversion();
void adcConvert();
void adcEndConvert();
uint32_t adcGetConvertBit();
uint32_t adcConverterGoertzel(uint16_t* data,uint32_t numSamples);
void adcConverterStop();
