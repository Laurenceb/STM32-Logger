#include "timer.h"
#include "gpio.h"
#include "Sensors/ppg.h"

/**
  * @brief  Configure the timer channels for PWM out on CRT board
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void setup_pwm(void) {
  /* -----------------------------------------------------------------------
    TIM Configuration: generate 2/3 PWM signals with different duty cycles:
    The TIMxCLK frequency is set to SystemCoreClock (Hz), to get TIMx counter
    clock at 72 MHz the Prescaler is computed as following:
     - Prescaler = (TIMxCLK / TIMx counter clock) - 1
    SystemCoreClock is set to 72 MHz

    The TIM3 is running at 11.905KHz: TIM3 Frequency = TIM4 counter clock/(ARR + 1)
                                                  = 72 MHz / 6047
    (with 239.5clk adc sampling -> 252adc clk/sample, and 12mhz adc clk this gives quadrature
    sampling)

  ----------------------------------------------------------------------- */
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={};
  TIM_OCInitTypeDef  	TIM_OCInitStructure={};
  GPIO_InitTypeDef	GPIO_InitStructure;
  /*Enable the Tim2 clk*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /*Enable the Tim4 clk*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /*Enable the Tim1 clk*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  #if BOARD>=3
  /*Enable the Tim3 clk*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  #endif
  //Setup the GPIO pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//reduced slew rate to reduce interference on the board
  GPIO_PinRemapConfig( GPIO_FullRemap_TIM2, ENABLE );//to B.10
  #if BOARD<3
	GPIO_InitStructure.GPIO_Pin = PWM0|PWM1|PWM2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
  #else
	GPIO_InitStructure.GPIO_Pin = PWM0|PWM2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = PWM1;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
  #endif		

  TIM_DeInit(TIM1);
  TIM_DeInit(TIM2);
  #if BOARD>=3
  TIM_DeInit(TIM3);
  #endif
  TIM_DeInit(TIM4);
  /*Setup the initstructure*/
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 4;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  /* Time base configuration  - timer 4 as PWM0/2*/
  #if BOARD<3
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_ZERO;
  #else
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_TWO;	//PWM2 on revision 3 boards
  #endif
  /*setup 4*/
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  #if BOARD<3
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//Mode 1 on early board revisions
  /* 'PWM1' Mode configuration: Channel3 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  #else
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;	//As Board revisions >=3 using P channel mosfet drivers, so inverted levels
  #endif
  /* PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /* TIM4 enable preload */
  TIM_ARRPreloadConfig(TIM4, ENABLE);


  /*Now setup timer2 as PWM1/0*/
  #if BOARD<3
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_ONE;
  #else
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_ZERO;
  #endif
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//Same as timer4
  /* PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* TIM2 enable preload */
  TIM_ARRPreloadConfig(TIM2, ENABLE);


  #if BOARD>=3
  /*Now setup timer3 as PWM1*/
  TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_ONE;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	//Same as timer4
  /* PWM1 Mode configuration: Channel1 */
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* TIM3 enable preload */
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  #endif 

  /*Now we setup the master/slave to orthogonalise the timers*/
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;//No signal output to pins
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//Normal PWM mode for gating
  #if BOARD>=3
  //Timers2 and 4 are slaved off timer3
  TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);//This is purely a syncronisation option applied to the master
  TIM_OCInitStructure.TIM_Pulse = GATING_PERIOD_ZERO;	//Macros used to define these
  TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_OC2Ref);
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);		//Tim3,ch2 used for gating
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);//This is purely a syncronisation option applied to the master
  TIM_SelectInputTrigger(TIM2,TIM_TS_ITR2);		//Tim2 off tim3
  TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Gated);	//Tim2 is gated by the tim3 channel2 input
  #endif
  //Timer4 is slaved off timer2 using channel1 output compare on both board revisions, providing we have at least 2 PPG channels
  #if BOARD>=3
  TIM_OCInitStructure.TIM_Pulse = GATING_PERIOD_ONE;	//Defined in header file
  #else
  TIM_OCInitStructure.TIM_Pulse = GATING_PERIOD_ZERO;	//Defined in header file
  #endif
  TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_OC1Ref);
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);		//Tim2,ch1 used for gating
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  #if PPG_CHANNELS>1
  //TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
  TIM_SelectInputTrigger(TIM4,TIM_TS_ITR1);		//Tim4 off tim2
  TIM_SelectSlaveMode(TIM4,TIM_SlaveMode_Gated);	//Tim4 is gated by the tim2 channel1 input
  #endif

  /*We enable all the timers at once with interrupts disabled*/
  __disable_irq();
  #if BOARD<3
    #if PPG_CHANNELS>1
      TIM_Cmd(TIM4, ENABLE);
    #endif
    TIM_Cmd(TIM2, ENABLE);
  #else
    #if PPG_CHANNELS>2
      TIM4->CNT=PWM_PERIOD_CENTER/2;//This causes the third timer to be in antiphase, giving reduce peak ADC signal
      TIM_Cmd(TIM4, ENABLE);
    #endif
    #if PPG_CHANNELS>1
      TIM_Cmd(TIM2, ENABLE);
    #endif
    TIM_Cmd(TIM3, ENABLE);
  #endif
  __enable_irq();

  /*Now setup timer1 as motor control */
  //note no prescaler
  TIM_TimeBaseStructure.TIM_Period = 2047;//gives a slower frequency - 35KHz, meeting Rohm BD6231F spec, and giving 11 bits of res each way
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//Make sure we have mode1 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//Enable output
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
  * Note: this could be improved, atm it just uses some macros and is hardcoded for the timers, but making it more flexible would need arrays of pointers
  * (need to use different timers for each output) 
  *//*
void Tryfudge(uint32_t* Fudgemask) {
	if((*Fudgemask)&(uint32_t)0x01 && TIM2->CNT<(FUDGED_PWM_PERIOD(0)-2)) {//If the second bit is set, adjust the first timer in the list if it is safe to do so
		TIM2->CR1 &= ~TIM_CR1_ARPE;//Disable reload buffering so we can load directly
		TIM2->ARR = FUDGED_PWM_PERIOD(0);//Load reload register directly
		TIM2->CR1 |= TIM_CR1_ARPE;//Enable buffering so we load buffered register
		TIM2->ARR = NORMAL_PWM_PERIOD(0);//Load the buffer, so the pwm period returns to normal after 1 period
		*Fudgemask&=~(uint32_t)0x01;//Clear the bit
	}
	if((*Fudgemask)&(uint32_t)0x04 && TIM4->CNT<(FUDGED_PWM_PERIOD(2)-2)) {//If the first bit is set, adjust the first timer in the list if it is safe to do so
		TIM4->CR1 &= ~TIM_CR1_ARPE;//Disable reload buffering so we can load directly
		TIM4->ARR = FUDGED_PWM_PERIOD(2);//Load reload register directly
		TIM4->CR1 |= TIM_CR1_ARPE;//Enable buffering so we load buffered register
		TIM4->ARR = NORMAL_PWM_PERIOD(2);//Load the buffer, so the pwm period returns to normal after 1 period
		*Fudgemask&=~(uint32_t)0x04;//Clear the bit
	}
}*/

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
