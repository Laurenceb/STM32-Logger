#include "timer.h"
#include "gpio.h"

/**
  * @brief  Configure the timer channels for PWM out on CRT board
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void setup_pwm(void) {
  /* -----------------------------------------------------------------------
    TIM4 Configuration: generate 2 PWM signals with 2 different duty cycles:
    The TIM4CLK frequency is set to SystemCoreClock (Hz), to get TIM4 counter
    clock at 72 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz

    The TIM4 is running at 11.905KHz: TIM4 Frequency = TIM4 counter clock/(ARR + 1)
                                                  = 4.5 MHz / 378
    (with 239.5clk adc sampling -> 252adc clk/sample, and 12mhz adc clk this gives quadrature
    sampling)
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 2.6%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 2.6%
  ----------------------------------------------------------------------- */
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  /*Enable the Tim3 clk*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /*Enable the Tim4 clk*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /*Enable the Tim1 clk*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  TIM_DeInit(TIM1);
  TIM_DeInit(TIM3);
  TIM_DeInit(TIM4);
  /* Prescaler of 16 times*/
  uint16_t PrescalerValue = 15;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  /*setup 4*/
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /*Setup the initstructure*/
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 10;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 

  /* PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* TIM4 enable counter */
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);


  /*Now setup timer3 as PWM0*/
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD3;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//same as timer4
  /* PWM1 Mode configuration: Channel2 */
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* TIM3 enable counter */
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  /*Now setup timer1 as motor control */
  PrescalerValue = 0;//no prescaler
  TIM_TimeBaseStructure.TIM_Period = 2047;//gives a slower frequency - 35KHz, meeting Rohm BD6231F spec, and giving 11 bits of res each way
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//These settings need to be applied on timers 1 and 8                 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);//same as timer4 
  /* PWM1 Mode configuration: Channel1 */
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);	//Needs to be applied on 1 and 8
  /* TIM1 enable counter */
  TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_Cmd(TIM1, ENABLE); 
  Set_PWM_Motor(0);			//Make sure motor off
}

/**
  * @brief Try to correct the timer phase by adjusting the reload register for one pwm cycle
  * @param Pointer to unsigned 32 bit integer bitmask for the timers to be corrected
  * @retval none
  * Note: this could be improved, atm it just uses some macros and is hardcoded for timer3 only
  * (need to use different timers for each output) 
  */
void Tryfudge(uint32_t* Fudgemask) {
	while(TIM3->CNT>=PWM_FUDGE3-1);
	if((*Fudgemask)&(uint32_t)1 /*&& TIM3->CNT<PWM_FUDGE3*/) {//If the first bit is set, adjust the first timer in the list if it is safe to do so
		TIM_ARRPreloadConfig(TIM3, DISABLE);//Disable reload buffering so we can load directly
		TIM_SetAutoreload(TIM3, PWM_FUDGE3);//Load reload register directly
		TIM_ARRPreloadConfig(TIM3, ENABLE);//Enable buffering so we load buffered register
		TIM_SetAutoreload(TIM3, PWM_PERIOD3);//Load the buffer, so the pwm period returns to normal after 1 period
		*Fudgemask&=~(uint32_t)1;//Clear the bit
	}
	//Other timers could go here at some point
}

/**
  * @brief  Configure the timer channel for PWM out to pump motor, -ive duty turns on valve
  * @param  None
  * @retval None
  * setting duty=0 gives idle state
  */
void Set_Motor(int16_t duty) {
	duty=(duty>MAX_DUTY)?MAX_DUTY:duty;	//enforce limits on range
	if(duty<0) {//We are dumping with the solenoid valve
		SET_SOLENOID(1);
		Set_PWM_Motor(0);
	}
	else {//We are driving the pump
		SET_SOLENOID(0);
		Set_PWM_Motor(duty);
	}
}
