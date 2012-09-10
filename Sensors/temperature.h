#include "stm32f10x.h"
#include "../i2c_int.h"

//These figures are from the F103 datasheet - offset has a +-20C error
//So best to do a single point cal
//Gain has a +-6% error, so if we cal at 25C, we have +-1C error from 8 to 42C
//Otherwise we need two point calibration
#define TEMPERATURE_GAIN ((float)0.18736)
#define TEMPERATURE_OFFSET ((float)357.6) /*Theoretical value from datasheet*/

//The prototype sensors use Murata NCP18XH103F03RB, 10K at 25C, Beta=3380K
//In a potential divider with 10K, gives a linear fit with good match (~0.2C error)
//between 20C and 40C with ;
#define THERMISTOR_GAIN	((float)-114.0/4096.0)
#define THERMISTOR_OFFSET ((float)-81.9)/* Note these are theoretical values from datasheet*/

float convert_die_temp(uint16_t adcval);

float convert_tmp102_temp(uint16_t adcval);
#if BOARD>=3
float convert_thermistor_temp(uint16_t adcval);
#endif
extern volatile float Device_Temperature;//CPU temperature monitoring, if supported, converted to centigrade

extern volatile uint16_t TMP102_Data_Buffer;
extern volatile float TMP102_Reported_Temperature;//Global used to pass compensated data
#define GET_TMP_TEMPERATURE convert_tmp102_temp(TMP102_Data_Buffer)

#define TMP102_BUFFER_SIZE 128		/*enough for 128 samples or 1280ms*/
