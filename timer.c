#include "timer.h"

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

    The TIM4 is running at 214.29KHz: TIM4 Frequency = TIM4 counter clock/(ARR + 1)
                                                  = 72 MHz / 336
    (with 1.5clk adc -> 14adc clk/sample, and 12mhz adc clk this gives quadrature
    sampling)
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  /* No prescaler */
  uint16_t PrescalerValue = 0;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 336;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /*Enable the Tim4 clk*/
  RCC_APB2PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /*Now setup timer3 as PWM0*/
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//same as timer4
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /*Enable the Tim3 clk*/
  RCC_APB2PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

}
