#include "stm32f10x.h"
#include "../timer.h"

#define PPG_CHANNELS 3

#define PPG_NO_SUBSAMPLES 12
//#define TARGET_ADC 1376256/(3*PPG_CHANNELS)/*Target 67% of ADC range used by the pwm led signal*/
#if PPG_CHANNELS==1
	#define TARGET_ADC 269746176UL*2/(3)/*Target 67% of ADC range used by the pwm led signal*/
#else
	#define TARGET_ADC 269746176UL*2/(3*sqrt(PPG_CHANNELS+2))/*This equation is true for arbitrary number of channels with correct phasing*/
#endif

#define PPG_BUFFER_SIZE 128
#define ADC_BUFFER_SIZE 4608

#define PWM_STEP_LIM  100		/* A 1% or less shift per iteration causes termination */

#define PPG_SAMPLE_RATE (float)62.004   /* 72000000/6/(12.5+1.5)/72/16/12  */

#define STEP_SIN {0,1,3,4,5,6,7,9,10,11,11,12,13,14,14,14,15,15,15,15,15,14,14,14,13,12,11,11,10,9,7,6,5,4,3,1,0,-1,-3,-4,-5,-6,-7,-9,-10,-11,-11,-12,\
-13,-14,-14,-14,-15,-15,-15,-15,-15,-14,-14,-14,-13,-12,-11,-11,-10,-9,-8,-6,-5,-4,-3,-1}
#define STEP_COS {15,15,15,14,14,14,13,12,11,11,10,9,7,6,5,4,3,1,0,-1,-3,-4,-5,-6,-7,-9,-10,-11,-11,-12,\
-13,-14,-14,-14,-15,-15,-15,-15,-15,-14,-14,-14,-13,-12,-11,-11,-10,-9,-8,-6,-5,-4,-3,-1,0,1,3,4,5,6,7,9,10,11,11,12,13,14,14,14,15,15}

extern volatile float Last_PPG_Values[PPG_CHANNELS];//Last values from the PPG decoders, useful for brightness control

void PPG_LO_Filter(volatile uint16_t* buff);
uint16_t PPG_correct_brightness(uint32_t Decimated_value, uint16_t PWM_value);
float PWM_Linear(uint16_t PWM_value);
