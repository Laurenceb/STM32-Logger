#include "stm32f10x.h"
#pragma once
#include "Util/fat_fs/inc/ff.h"
#include "Util/buffer.h"
#include "Util/delay.h"
#include "Sensors/ppg.h"
#include "timer.h"

//externs for all the globals
extern volatile uint8_t Pressure_control;
extern volatile float Pressure_Setpoint;

extern volatile uint32_t Millis;

extern volatile uint8_t System_state_Global;
extern volatile uint8_t Sensors;

#define PRE_SIZE 1000000ul	/*Preallocate size*/

#define SYSTEM_STATES 1		/*Number of different control states- atm just brightness control implimented*/

#define WATCHDOG_TIMEOUT 3000	/*4 second timeout - enough for the uSD card to block its max time and a bit*/

//Pressure controller constants - a PI controller
#define ITERATION_RATE 100
#define I_HOLDOFF ITERATION_RATE/2			/*means that there is 500ms allowed for the pump to turn on and stabilise before I enabled*/
#define PRESSURE_I_CONST 0.05*(float)(PWM_RES/ITERATION_RATE)/*means an error of 1PSI for 1 second will case a 5% duty cycle shift*/
#define PRESSURE_I_LIM 0.8*(float)PWM_RES		/*means integral term can never cause more than 80% shift in duty cycle*/
#define PRESSURE_P_CONST 0.01*(float)PWM_RES		/*means a 1PSI error gives 1% duty cycle pwm to the motor*/
#define PRESSURE_D_CONST -0.001*(float)(PWM_RES*ITERATION_RATE)/*means a change of 1PSI/second will decrease pwm duty by 0.1%*/
#define PRESSURE_MARGIN 0.3				/*means a pressure within 0.3PSI of zero will turn off the dump valve if setpoint -ive*/

//Battery specific config goes here
#define BATTERY_STARTUP_LIMIT 3.7 /*Around 25% capacity remaining for lithium polymer at 25C slow discharge*/
#define MINIMUM_VOLTAGE 3.0	/* A single lithium polymer cell*/

//Sensors
enum {PRESSURE_HOSE=0,TEMPERATURE_SENSOR, PPG_SENSOR_ZERO, PPG_SENSOR_ONE, PPG_SENSOR_TWO, THERMISTOR_SENSOR};
#define PPG_SENSORS ((1<<PPG_SENSOR_ZERO)|(1<<PPG_SENSOR_ONE)|(1<<PPG_SENSOR_TWO))

//function prototypes
void __fat_print_char(char c);
void __str_print_char(char c);
uint8_t detect_sensors(void);
//buffer globals
extern volatile buff_type Buff[PPG_CHANNELS];
extern volatile buff_type Pressures_Buffer;		//Data from pressure sensor
extern volatile buff_type Temperatures_Buffer;		//Data from temperature sensor
extern volatile buff_type Thermistor_Buffer;		//Data from patient mounted thermistor for temperature sensing
//fatfs globals
extern volatile uint8_t file_opened;
extern FIL FATFS_logfile;
