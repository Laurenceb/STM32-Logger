//Dactyl project v1.0

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "watchdog.h"

/**
  * @brief  Configures the independant watchdog
  * @param  Timeout in milliseconds
  * @retval None
  */
void Watchdog_Config(uint16_t timeout_ms) {
	#ifdef LSI_TIM_MEASURE
		/* Enable the LSI OSC */
		RCC_LSICmd(ENABLE);
		/* Wait till LSI is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
		{}  
		/* TIM Configuration -------------------------------------------------------*/
		TIM5_ConfigForLSI();  
		/* Wait until the TIM5 get 2 LSI edges */
		while(CaptureNumber != 2)
		{}
		/* Disable TIM5 CC4 Interrupt Request */
		TIM_ITConfig(TIM5, TIM_IT_CC4, DISABLE);
	#endif
	/* IWDG timeout equal to timeout ms (the timeout may varies due to LSI frequency dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value to obtain 250ms IWDG TimeOut.
	Counter Reload Value e.g. = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (LsiFreq/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
	So set timeout_ms*LsiFreq/(128*250)=1.25*timeout_ms=timeout_ms*5/4
	As LsiFreq=40000 Hz
	*/
	IWDG_SetReload(timeout_ms*5/4);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

uint8_t Watchdog_Reset_Detect(void) {
	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
		/* IWDGRST flag set */
 		/* Clear reset flags */
		RCC_ClearFlag();
		return 1;
  	}
	else {
		/* IWDGRST flag is not set */
		/* Turn off LED1 */
		return 0;
	}
}


#ifdef LSI_TIM_MEASURE
/**
  * @brief  Configures TIM5 to measure the LSI oscillator frequency.
  * @param  None
  * @retval None
  */
void TIM5_ConfigForLSI(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;

  /* Enable TIM5 clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  
  /* Enable the TIM5 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure TIM5 prescaler */
  TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);

  /* Connect internally the TM5_CH4 Input Capture to the LSI clock output */
  GPIO_PinRemapConfig(GPIO_Remap_TIM5CH4_LSI, ENABLE);
  
  /* TIM5 configuration: Input Capture mode ---------------------
     The LSI oscillator is connected to TIM5 CH4
     The Rising edge is used as active edge,
     The TIM5 CCR4 is used to compute the frequency value 
  ------------------------------------------------------------ */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
  /* TIM10 Counter Enable */
  TIM_Cmd(TIM5, ENABLE);

  /* Reset the flags */
  TIM5->SR = 0;
    
  /* Enable the CC4 Interrupt Request */  
  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);  
}
#endif
