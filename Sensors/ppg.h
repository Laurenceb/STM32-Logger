#include "stm32f10x.h"

#define PPG_CHANNELS 2

#define TARGET_ADC 1376256/2	/*Target 50% of ADC range used by the pwm led signal*/
#define PPG_BUFFER_SIZE 512

#define PPG_SAMPLE_RATE (float)62.004    /* 72000000/6/(12.5+239.5)/64/12  */

void PPG_LO_Filter(uint16_t* buff);
uint16_t PPG_correct_brightness(uint32_t Decimated_value, uint16_t PWM_value);
float PWM_Linear(uint16_t PWM_value);
