#include "stm32f10x.h"
#include "Sensors/ppg.h"

#define CRT_PPG_ADC_CHAN 8	/* 4 is also possible - second PPG channel PORTB0,PORTA4 */
#if BOARD<3
	#define SECOND_ADC_CHAN 4
	#define PRESSURE_ADC_CHAN 1/* The differential pressure sensor */
	#define BATTERY_ADC_CHAN 9/* The battery voltage monitoring (alternatively third sensor input)*/
#else
	#define SECOND_ADC_CHAN 9/* Usually used for a thermistor*/
	#define PRESSURE_ADC_CHAN 1/* The guage pressure sensor */
	#define BATTERY_ADC_CHAN 4/* The battery voltage monitoring */
	#define THERMISTOR_ADC_CHAN SECOND_ADC_CHAN/* Sensor thermistor */
#endif
#define ADC_BUFF_SIZE ADC_BUFFER_SIZE/* 64 samples * 2 for interleaving, * 2bytes/sample==256 */


#define SAMPLING_FACTOR	4096.0/6.6/* 1/2 factor pot divider on the frontend board - integrated on BOARD>=3 */

#define GET_BATTERY_VOLTAGE (float)readADC2(BATTERY_ADC_CHAN)/(SAMPLING_FACTOR)/* Macro to return battery voltage as a float using blocking regular conv*/

extern volatile uint16_t * ADC1_Convertion_buff;//malloc this

void ADC_Configuration(void);
void setADC2(uint8_t channel);
uint16_t readADC2(uint8_t channel);
uint16_t getADC2(void);
