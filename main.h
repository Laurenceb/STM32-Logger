#include "stm32f10x.h"
#pragma once
#include "Util/fat_fs/inc/ff.h"
#include "Util/buffer.h"
#include "timer.h"

//externs for all the globals
extern volatile uint8_t Pressure_control;
extern volatile float pressure_setpoint;
extern volatile float reported_pressure;

#define PRE_SIZE 1000000ul	/*Preallocate size*/

//Pressure controller constants - a PI controller
#define ITERATION_RATE 100
#define PRESSURE_I_CONST 0.1*(float)(PWM_RES/ITERATION_RATE)/*means an error of 1PSI for 1 second will case a 10% duty cycle shift*/
#define PRESSURE_I_LIM 0.2*(float)PWM_RES		/*means integral term can never cause more than 20% shift in duty cycle*/
#define PRESSURE_P_CONST 0.3*(float)PWM_RES		/*means a 1PSI error gives 30% duty cycle pwm to the motor*/

#define delay()						\
do {							\
  register unsigned int i;				\
  for (i = 0; i < 10000000; ++i)			\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)

//function prototypes
void __fat_print_char(char c);
void __str_print_char(char c);
//globals
extern volatile buff_type Buff;
//fatfs globals
extern volatile uint8_t file_opened;
extern FIL FATFS_logfile;
