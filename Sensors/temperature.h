#include "stm32f10x.h"

//These figures are from the F103 datasheet - offset has a +-20C error
//So best to do a single point cal
//Gain has a +-6% error, so if we cal at 25C, we have +-1C error from 8 to 42C
//Otherwise we need two point calibration
#define TEMPERATURE_GAIN (float)0.18736
#define TEMPERATURE_OFFSET (float)357.6 /*Theoretical value from datasheet*/

float convert_die_temp(uint16_t adcval);
