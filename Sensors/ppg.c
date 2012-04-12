#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "ppg.h"
#include "../adc.h"
#include "../Util/buffer.h"
#include "../main.h"
#include "../timer.h"

volatile uint32_t Last_PPG_Values[3];

//This should really be done with macros, but 64(decimate)*21(decimate)*14(adc clks)*6(adc clkdiv)=112896
// == 336*336 - so one change in the pwm reload value will give an orthogonal frequency to the baseband decimator 

/**
  * @brief  Run the baseband LO on the quadrature samples, then integrate and dump the baseband into indivdual LED bins
  * @param  Pointer to the output data buffer
  * @retval None
  * This will be called at 13.393KHz
  */
volatile uint8_t a_;

void PPG_LO_Filter(uint16_t* buff) {
	static uint16_t record[768];
	int32_t I=0,Q=0,a;			//I and Q integration bins, general purpose variable
	static uint8_t bindex;			//Baseband decimation index
	static int32_t Frequency_Bin[2][2];	//Only two frequencies in use atm - consisting of and I and Q component
	static uint32_t Fudgemask,store;
	Tryfudge(&Fudgemask);			//Try to correct timer phase here
	for(uint16_t n=0;n<ADC_BUFF_SIZE/4;) {	//buffer size/4 must be a multiple of 4
		I+=buff[n++];
		Q+=buff[n++];
		I-=buff[n++];
		Q-=buff[n++];
	}
	memcpy(&record[(uint16_t)bindex*64],buff,128);
	//Now run the "baseband" decimating filter(s)
	//No positive frequencies at the moment - they would go here TODO
	Frequency_Bin[0][0]+=I;Frequency_Bin[0][1]+=Q;//Add the I and Q directly into the zero frequency bin
	//Negative frequencie(s) go here, need to get to 0hz, so multiply by a +ive complex exponential
	a=Frequency_Bin[1][0];Frequency_Bin[1][0]=Frequency_Bin[1][0]*1774-Frequency_Bin[1][1]*1024;//Rotate the phasor in the bin - real here
	Frequency_Bin[1][1]=Frequency_Bin[1][1]*1773+a*1024;//complex here
	Frequency_Bin[1][1]>>=11;Frequency_Bin[1][0]>>=11;//divide by 2048
	Frequency_Bin[1][0]+=I;Frequency_Bin[1][1]+=Q;//I,Q is real,imaginary
	//End of decimating filters
	if(++bindex==12) {			//Decimation factor of 12 - 62.004Hz data output
		Last_PPG_Values[0]=(uint32_t)sqrt(pow((int64_t)Frequency_Bin[0][0],2)+pow((int64_t)Frequency_Bin[0][1],2));
		Last_PPG_Values[1]=(uint32_t)sqrt(pow((int64_t)Frequency_Bin[1][0],2)+pow((int64_t)Frequency_Bin[1][1],2));
		Add_To_Buffer(Last_PPG_Values[0],&(Buff[0]));
		Add_To_Buffer(Last_PPG_Values[1],&(Buff[1]));
		//Other frequencies corresponding to different LEDs could go here - use the array of buffers?
		memset(Frequency_Bin,0,sizeof(Frequency_Bin));//Zero everything
		bindex=0;			//Reset this
		Fudgemask|=1;			//Sets a TIM3 fudge as requested
		if((Last_PPG_Values[1]+10000)<store) {
			a_=1;
		}
		store=Last_PPG_Values[1];
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
			vals[channel]=PPG_correct_brightness(Last_PPG_Values[channel], pwm);
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
