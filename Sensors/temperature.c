#include "temperature.h"

volatile uint16_t TMP102_Data_Buffer;
volatile float Device_Temperature;
volatile float TMP102_Reported_Temperature;

float convert_die_temp(uint16_t adcval) {
	return ((float)adcval*TEMPERATURE_GAIN)-TEMPERATURE_OFFSET;
}

float convert_tmp102_temp(uint16_t adcval) {
	Flipbytes(adcval);			//Fix the endianess
	adcval>>=3;				//Move to right aligned
	if(adcval&(1<<12))			//If this bit is set, its negative
		return ((float)(adcval&0x0FFF)*0.0625);
	return (((float)adcval)*0.0625);	//TMP102 is 0.0625C/LSB
}

#if BOARD>=3
float convert_thermistor_temp(uint16_t adcval) {
	return ((float)adcval*THERMISTOR_GAIN)-THERMISTOR_OFFSET;
}
#endif
