#include "stm32f10x.h"
#include "../i2c_int.h"

//These figures are from the F103 datasheet - offset has a +-20C error
//So best to do a single point cal
//Gain has a +-6% error, so if we cal at 25C, we have +-1C error from 8 to 42C
//Otherwise we need two point calibration
#define TEMPERATURE_GAIN (float)0.18736
#define TEMPERATURE_OFFSET (float)357.6 /*Theoretical value from datasheet*/

float convert_die_temp(uint16_t adcval);

float convert_tmp102_temp(uint16_t adcval);

extern volatile float Device_Temperature;//CPU temperature monitoring, if supported

extern volatile uint16_t TMP102_Data_Buffer;
extern volatile float TMP102_Reported_Temperature;//Global used to pass compensated data
#define GET_TMP_TEMPERATURE convert_tmp102_temp(TMP102_Data_Buffer)

#define TMP102_BUFFER_SIZE 256		/*enough for 64 samples or 640ms*/
