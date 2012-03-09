#include "stm32f10x.h"

#define MAX_DUTY 2047/2	/*max duty cycle for the motor - dont want to risk going to 100% as it will overrev motor*/
#define PWM_RES MAX_DUTY/*motor pwm resolution*/

//The idea here is that there are 192 cycles of first pwm output (at quadrature) and 191 of second pwm (just below quadrature)
//192*378=72576 timer clocks 
#define PWM_PERIOD 377	/* 377+1=378 gives 11.905khz pwm with 72/16=4.5mhz clk - for quadrature sampling */

#define PWM_PERIOD3 379	/* 379+1=380 These two values are used to generate an orthogonal frequency for second LED channel */
#define PWM_FUDGE3  375 /* 380*191=72580, so we need 80-76=4 take off clks to correct phase after each integration bin is filled */

//Macros to link to the periph functions
#define Set_PWM_0(compare) TIM_SetCompare2(TIM3, compare)
#define Set_PWM_1(compare) TIM_SetCompare3(TIM4, compare)
#define Set_PWM_2(compare) TIM_SetCompare4(TIM4, compare)
#define Set_PWM_Motor(compare) TIM_SetCompare1(TIM1, compare)
//Configure the pwm
void setup_pwm(void);
//Fudge timers to fix phase offsets
void Tryfudge(uint32_t* Fudgemask);
//Set the motor
void Set_Motor(int16_t duty);
