#include "stm32f10x.h"

#define MAX_DUTY 2047/2	/*max duty cycle for the motor - dont want to risk going to 100% as it will overrev motor*/
#define PWM_RES MAX_DUTY/*motor pwm resolution*/

#define PWM_PERIOD 377	/*gives 11.905khz pwm with 72/16=4.5mhz clk - for quadrature sampling*/
//Macros to link to the periph functions
#define Set_PWM_0(compare) TIM_SetCompare2(TIM3, compare)
#define Set_PWM_1(compare) TIM_SetCompare3(TIM4, compare)
#define Set_PWM_2(compare) TIM_SetCompare4(TIM4, compare)
#define Set_PWM_Motor(compare) TIM_SetCompare1(TIM1, compare)
//Configure the pwm
void setup_pwm(void);
//Set the motor
void Set_Motor(int16_t duty);
