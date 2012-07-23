#include "stm32f10x.h"
#pragma once
#include "Sensors/ppg.h"
#define MAX_DUTY 2047*3/4	/*max duty cycle for the motor - dont want to risk going to 100% as it will overrev motor*/
#define PWM_RES MAX_DUTY/*motor pwm resolution*/

//The idea here is that there are 192 cycles of first pwm output (at quadrature) and 191 of second pwm (just below quadrature)
#define PWM_CYCLES_CENTER 192ul 
#define PWM_PERIOD_CENTER 378ul/* 378 gives 11.905khz pwm with 72/16=4.5mhz clk - for quadrature sampling */

#define PWM_PERIOD_CLKS (PWM_CYCLES_CENTER*PWM_PERIOD_CENTER)/* 192*378=72576 timer clocks */

#if(PPG_CHANNELS&0x01)	/* An odd number of enabled channels - number of channels goes 1,2,3 etc*/
	#pragma "Odd number of PPG Channels"
	#define PWM_PERIOD_BASE (PWM_PERIOD_CENTER-PPG_CHANNELS+1)
	#define PWM_CYCLES_BASE (PWM_CYCLES_CENTER+(PPG_CHANNELS-1)/2)
#else			/* An even number */
	#pragma "Even number of PPG Channels"
	#define PWM_PERIOD_BASE (PWM_PERIOD_CENTER-PPG_CHANNELS+2)
	#define PWM_CYCLES_BASE (PWM_CYCLES_CENTER+(PPG_CHANNELS-2)/2) 
#endif

//Macro to give correct pwm periods - channels are numbered 0,1,2 etc
#define NORMAL_PWM_PERIOD(n) (PWM_PERIOD_BASE+(2*n)-1)
#define FUDGED_PWM_PERIOD(n) (PWM_PERIOD_CLKS-(NORMAL_PWM_PERIOD(n)+1)*(PWM_CYCLES_BASE-n)-1)

//Macros for fudging timers
#define FUDGE_ALL_TIMERS ((1<<PPG_CHANNELS)-1)

//Macros to link to the periph functions
#if BOARD<3
	#define Set_PWM_0(compare) TIM_SetCompare3(TIM4, compare)
	#define Set_PWM_1(compare) TIM_SetCompare3(TIM2, compare)
	#define Set_PWM_2(compare) TIM_SetCompare4(TIM4, compare)
	#define Set_PWM_Motor(compare) TIM_SetCompare1(TIM1, compare)
	#define Get_PWM_0()  TIM4->CCR3
	#define Get_PWM_1()  TIM2->CCR3
	#define Get_PWM_2()  TIM4->CCR4
#else			/* Note that on version 3+ of the PCB, PFET output drivers mean output is inverted - we use PWM mode2*/
	#define Set_PWM_0(compare) TIM_SetCompare3(TIM2, compare)
	#define Set_PWM_1(compare) TIM_SetCompare1(TIM3, compare)
	#define Set_PWM_2(compare) TIM_SetCompare4(TIM4, compare)
	#define Set_PWM_Motor(compare) TIM_SetCompare1(TIM1, compare)
	#define Get_PWM_0()  TIM2->CCR3
	#define Get_PWM_1()  TIM3->CCR1
	#define Get_PWM_2()  TIM4->CCR4
#endif
//Configure the pwm
void setup_pwm(void);
//Fudge timers to fix phase offsets
void Tryfudge(uint32_t* Fudgemask);
//Set the motor
void Set_Motor(int16_t duty);
