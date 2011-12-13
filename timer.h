#include "stm32f10x.h"

#define PWM_PERIOD 336	/*gives 214khz pwm with 72mhz clk - for quadrature sampling*/
//Macros to link to the periph functions
#define Set_PWM_0(compare) TIM_SetCompare2(TIM3, compare)
#define Set_PWM_1(compare) TIM_SetCompare3(TIM4, compare)
#define Set_PWM_2(compare) TIM_SetCompare4(TIM4, compare)
#define Set_PWM_Motor(compare) TIM_SetCompare1(TIM1, compare)
//Configure the pwm
void setup_pwm(void);
//Set the motor
void Set_Motor(int16_t duty);
