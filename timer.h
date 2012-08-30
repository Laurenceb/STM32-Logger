#include "stm32f10x.h"
#pragma once
#include "Sensors/ppg.h"
#define MAX_DUTY 2047*3/4	/*max duty cycle for the motor - dont want to risk going to 100% as it will overrev motor*/
#define PWM_RES MAX_DUTY	/*motor pwm resolution*/

#define PWM_PERIOD_TWO 6047ul//5982ul
#define PWM_PERIOD_ONE 6047ul
#define PWM_PERIOD_ZERO 5951ul

#define GATING_PERIOD_ZERO 5983ul
#define GATING_PERIOD_ONE  5921ul

#define PWM_PERIOD_CENTER PWM_PERIOD_ONE
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
