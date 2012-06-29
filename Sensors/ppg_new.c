#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "ppg.h"
#include "../adc.h"
#include "../Util/buffer.h"
#include "../main.h"
#include "../timer.h"

volatile float Last_PPG_Values[3];

//This should really be done with macros, but 64(decimate)*21(decimate)*14(adc clks)*6(adc clkdiv)=112896
// == 336*336 - so one change in the pwm reload value will give an orthogonal frequency to the baseband decimator 

/**
  * @brief  Run the baseband LO on the quadrature samples, then integrate and dump the baseband into indivdual LED bins
  * @param  Pointer to the output data buffer
  * @retval None
  * This will be called at 744.048Hz
  */
void PPG_LO_Filter(volatile uint16_t* buff) {
	static uint8_t bindex;			//Baseband decimation index
	static int32_t Frequency_Bin[3][2];	//Only three frequencies in use atm - consisting of and I and Q component
	static uint32_t Fudgemask;
	static const int8_t sinusoid[72]=STEP_SIN,cosinusoid[72]=STEP_COS;//Lookup tables
	int32_t I=0,Q=0,a;			//I and Q integration bins, general purpose variables
	Tryfudge(&Fudgemask);			//Try to correct timer phase here
	for(uint16_t n=0;n<ADC_BUFF_SIZE/4;) {	//buffer size/4 must be a multiple of 4
		for(uint8_t m=0;m<72;m++,n++) {	//Loop through the 72 sample lookup
			I+=(int16_t)buff[n]*(int16_t)cosinusoid[m];
			Q+=(int16_t)buff[n]*(int16_t)sinusoid[m];
		}
	}
	//Now run the "baseband" decimating filter(s)
	//Positive frequency
	#if PPG_CHANNELS>=3
	a=Frequency_Bin[2][0];
	Frequency_Bin[2][0]=Frequency_Bin[2][0]*7+Frequency_Bin[2][1]*4;//Rotate the phasor in the bin - real here (~-30degree rotation)
	Frequency_Bin[2][1]=Frequency_Bin[2][1]*7-a*4;//complex here
	Frequency_Bin[2][1]>>=3;Frequency_Bin[2][0]>>=3;//divide by 8
	#endif
	//Zero frequency - i.e. directly on quadrature 
	//nothing to do to this bin
	//Negative frequencie(s) go here, need to get to 0hz, so multiply bin by a +ive complex exponential
	a=Frequency_Bin[2][0];
	Frequency_Bin[2][0]=Frequency_Bin[2][0]*7-Frequency_Bin[2][1]*4;//Rotate the phasor in the bin - real here (~30degree rotation)
	Frequency_Bin[2][1]=Frequency_Bin[2][1]*7+a*4;//complex here
	Frequency_Bin[2][1]>>=3;Frequency_Bin[2][0]>>=3;//divide by 8
	#if PPG_CHANNELS>3
	#error "Unsupported number of channels - decoder error"
	#endif
	//Add the I and Q directly into the bins
	for(uint8_t n=0;n<PPG_CHANNELS;n++) {
		Frequency_Bin[n][0]+=I;Frequency_Bin[n][1]+=Q;//I,Q is real,imaginary
	}
	//End of decimating filters
	if(++bindex==PPG_NO_SUBSAMPLES) {	//Decimation factor of 12 - 62.004Hz data output
		Last_PPG_Values[0]=sqrtf(((float)Frequency_Bin[0][0]*(float)Frequency_Bin[0][0])+((float)Frequency_Bin[0][1]*(float)Frequency_Bin[0][1]));
		Last_PPG_Values[1]=sqrtf(((float)Frequency_Bin[1][0]*(float)Frequency_Bin[1][0])+((float)Frequency_Bin[1][1]*(float)Frequency_Bin[1][1]));
		Add_To_Buffer(((uint32_t)Last_PPG_Values[0])>>8,&(Buff[0]));//There will always be at least 8 bits on noise, so shift out the mess
		Add_To_Buffer(((uint32_t)Last_PPG_Values[1])>>8,&(Buff[1]));
		//Other frequencies corresponding to different LEDs could go here - use the array of buffers?
		memset(Frequency_Bin,0,sizeof(Frequency_Bin));//Zero everything
		bindex=0;			//Reset this
		Fudgemask|=1;			//Sets a TIM3 fudge as requested
	}
}

/**
  * @brief  Corrects PWM values to get the ADC input in the correct range
  * @param  none
  * @retval none
*/
void PPG_Automatic_Brightness_Control(void) {
	uint8_t channel;
	uint16_t vals[3]={0,0,0};
	uint16_t old_vals[3];			//This function iterates until the PWM duty correction falls below a limit set in header
	do {
		memcpy(old_vals,vals,sizeof(old_vals));//Copy over to the old values
		for(channel=0;channel<PPG_CHANNELS;channel++) {	//Loop through the channels
			uint16_t pwm=Get_PWM_0();
			if(channel==1)
				pwm=Get_PWM_1();
			else if(channel==2)
				pwm=Get_PWM_2();//Retreives the set pwm for this channel
			vals[channel]=PPG_correct_brightness((uint32_t)Last_PPG_Values[channel], pwm);
		}
		//Apply the pwm duty correction here
		Set_PWM_2(vals[2]);
		Set_PWM_1(vals[1]);
		Set_PWM_0(vals[0]);
		uint32_t time=Millis;	//Store the system time here
		time+=(uint32_t)(4000.0/PPG_SAMPLE_RATE);//four samples
		while(Millis<time);	//Delay for a couple of PPG samples to set the analogue stabilise	
	}while((abs(vals[0]-old_vals[0])>PWM_STEP_LIM)||(abs(vals[1]-old_vals[1])>PWM_STEP_LIM)||(abs(vals[2]-old_vals[2])>PWM_STEP_LIM));
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
	corrected_pwm*=(float)(TARGET_ADC)/(float)Decimated_value;//This is the linearised pwm value required to get target amplitude
	corrected_pwm=(corrected_pwm>1.0)?1.0:corrected_pwm;//Enforce limit on range to 100%
	return ((asinf(corrected_pwm)/M_PI)*PWM_PERIOD);//Convert back to a PWM period value
}

/**
  * @brief  Output a linearised value in range 0 to 1 from a PWM duty cycle
  * @param  PWM duty cycle
  * @retval A linearised value as a float in the range 0 to 1
  */
float PWM_Linear(uint16_t PWM_value) {
	return sinf(((float)PWM_value/(float)PWM_PERIOD)*M_PI);//returns the effecive sinusoidal amplitude in range 0-1
}
