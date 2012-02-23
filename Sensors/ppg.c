#include <math.h>
#include <string.h>
#include "ppg.h"
#include "../adc.h"
#include "../Util/buffer.h"
#include "../main.h"
#include "../timer.h"

//This should really be done with macros, but 64(decimate)*21(decimate)*14(adc clks)*6(adc clkdiv)=112896
// == 336*336 - so one change in the pwm reload value will give an orthogonal frequency to the baseband decimator 

/**
  * @brief  Run the baseband LO on the quadrature samples, then integrate and dump the baseband into indivdual LED bins
  * @param  Pointer to the output data buffer
  * @retval None
  * This will be called at 13.393KHz
  */
void PPG_LO_Filter(uint16_t* buff) {
	int32_t I=0,Q=0;	//I and Q integration bins
	static uint8_t bindex;	//Baseband decimation index
	static int32_t Frequency_Bin[1][2];//Only one frequency in use atm - consisting of and I and Q component
	for(uint16_t n=0;n<ADC_BUFF_SIZE/4;) {//buffer size/4 must be a multiple of 4
		I+=buff[n++];
		Q+=buff[n++];
		I-=buff[n++];
		Q-=buff[n++];
	}
	//Now run the "baseband" decimating filter(s)
	Frequency_Bin[0][0]+=I;Frequency_Bin[0][1]+=Q;//Add the I and Q directly into the zero frequency bin
	//Other Bins for +ive or -ive frequencies go here - use a lookup table for the sin/cos samples
	I=0;Q=0;//Zero the quadrature sampling decimation bins
	if(++bindex==12) {//Decimation factor of 12 - 62.004Hz data output
		Add_To_Buffer((uint32_t)sqrt(pow((int64_t)Frequency_Bin[0][0],2)+pow((int64_t)Frequency_Bin[0][1],2)),&Buff);
		//Other frequencies corresponding to different LEDs could go here - use different buffers maybe?
		memset(Frequency_Bin,0,sizeof(Frequency_Bin));//Zero everything
		bindex=0;//reset this
	}
}

/**
  * @brief  Output a corrected PWM value to get the ADC input in the correct range
  * @param  Output sample from the decimator, present PWM duty cycle value
  * @retval A new corrected duty cycle value
  * This will be called from the main code between pressure applications and timed refills
  * If more leds are added at different pwm frequencies, then we need to take the sum of Decimated values and scale
  * To avoid clipping of the frontend
  */
uint16_t PPG_correct_brightness(uint32_t Decimated_value, uint16_t PWM_value) {
	//2^adc_bits*samples_in_half_buffer/4*baseband_decimator
	//(2^12)*(64/4)*21 == 1376256 == 2*target_decimated_value TODO impliment this with macros full - atm just TARGET_ADC
	float corrected_pwm=PWM_Linear(PWM_value);
	corrected_pwm*=TARGET_ADC/Decimated_value;//This is the linearised pwm value required to get target amplitude
	corrected_pwm=(corrected_pwm>1.0)?1.0:corrected_pwm;//Enforce limit on range to 100%
	return ((asin(corrected_pwm)/M_PI)*(2*PWM_PERIOD));//Convert back to a PWM period value
}

/**
  * @brief  Output a linearised value in range 0 to 1 from a PWM duty cycle
  * @param  PWM duty cycle
  * @retval A linearised value as a float in the range 0 to 1
  */
float PWM_Linear(uint16_t PWM_value) {
	return sin((float)(PWM_value/PWM_PERIOD)*M_PI/2.0);//returns the effecive sinusoidal amplitude in range 0-1
}
