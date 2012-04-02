#include "stm32f10x.h"
#pragma once
#include "Util/fat_fs/inc/ff.h"
#include "Util/buffer.h"
#include "Sensors/ppg.h"
#include "timer.h"

//externs for all the globals
extern volatile uint8_t Pressure_control;
extern volatile float pressure_setpoint;
extern volatile float reported_pressure;

extern volatile uint32_t Millis;

extern volatile float Device_Temperature;

#define PRE_SIZE 1000000ul	/*Preallocate size*/

//Pressure controller constants - a PI controller
#define ITERATION_RATE 100
#define I_HOLDOFF ITERATION_RATE/2			/*means that there is 500ms allowed for the pump to turn on and stabilise before I enabled*/
#define PRESSURE_I_CONST 0.05*(float)(PWM_RES/ITERATION_RATE)/*means an error of 1PSI for 1 second will case a 5% duty cycle shift*/
#define PRESSURE_I_LIM 0.8*(float)PWM_RES		/*means integral term can never cause more than 80% shift in duty cycle*/
#define PRESSURE_P_CONST 0.01*(float)PWM_RES		/*means a 1PSI error gives 1% duty cycle pwm to the motor*/
#define PRESSURE_D_CONST -0.001*(float)(PWM_RES*ITERATION_RATE)/*means a change of 1PSI/second will decrease pwm duty by 0.1%*/
#define PRESSURE_MARGIN 0.3				/*means a pressure within 0.3PSI of zero will turn off the dump valve if setpoint -ive*/

#define delay(x)					\
do {							\
  register unsigned int i;				\
  for (i = 0; i < x; ++i)			\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)

//function prototypes
void __fat_print_char(char c);
void __str_print_char(char c);
//buffer globals
extern volatile buff_type Buff[PPG_CHANNELS];
extern volatile buff_type Button_Buffer;
//fatfs globals
extern volatile uint8_t file_opened;
extern FIL FATFS_logfile;
