#include "stm32f10x.h"
#include "../timer.h"

#define PPG_CHANNELS 2

#define PPG_NO_SUBSAMPLES 12
#define TARGET_ADC 1376256/(3*PPG_CHANNELS)/*Target 67% of ADC range used by the pwm led signal*/

#define PPG_BUFFER_SIZE 256

#define PWM_STEP_LIM  PWM_PERIOD/100

#define PPG_SAMPLE_RATE (float)62.004    /* 72000000/6/(12.5+239.5)/64/12  */

#define 18_STEP_SIN {0,5,10,13,15,15,13,10,5,0,-5,-10,-13,-15,-15,-13,-10,-5}

extern volatile uint32_t Last_PPG_Values[3];//Last values from the PPG decoders, useful for brightness control

void PPG_LO_Filter(uint16_t* buff);
uint16_t PPG_correct_brightness(uint32_t Decimated_value, uint16_t PWM_value);
float PWM_Linear(uint16_t PWM_value);
