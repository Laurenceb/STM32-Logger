#pragma once
#include <stdio.h>
#include "stm32f10x.h"
#include "gpio.h"
#include "Sensors/temperature.h"

//Datatypes
typedef struct{
	uint8_t error;
	uint8_t job;
} I2C_Error_Type;

typedef struct{
	const uint8_t address;	//device address for this job
	const uint8_t direction;//direction (I2C_Direction_Transmitter or I2C_Direction_Receiver)
	const uint8_t bytes;	//number of bytes to read/write
	const uint8_t subaddress;//register subaddress for the device - set to 0xFF if no subaddress used (direct read/write)
	volatile uint8_t* data_pointer;	//points to the data
} I2C_Job_Type;

//Globals
extern volatile uint32_t Jobs,Completed_Jobs;	//used for task control (only ever access this from outside for polling Jobs/Reading Completed_Jobs)
extern volatile I2C_Error_Type I2C1error;	//used to store error state
//Macros

//Sensor specific defines
#define TMP102_ADDR 0x92
#define TMP102_DATA 0x00			/*sub address where data begins*/
#define TMP102_CONF 0x01			/*sub address for configuration*/
//Number of jobs
#define I2C_NUMBER_JOBS 2
//Setup for the core sensors - other sensors have setup in their respective header files - look in /sensors 
#define TMP102_SETUP {0xE1,0x30} /*configure the TMP102 sensor for one shot, 13bit output mode*/
//Jobs structure initialiser 
#define I2C_JOBS_INITIALISER {\
{TMP102_ADDR,I2C_Direction_Receiver,2,TMP102_DATA,NULL}, /*Read the TMP102 temperature*/\
{TMP102_ADDR,I2C_Direction_Transmitter,2,TMP102_CONF,TMP102_setup}, /*Setup a single TMP102 conversion*/\
}
//Job identifiers used to run the accel downsampler, trigger jobs from the EXTI, and trigger the Kalman task
#define TMP102_READ  0
#define TMP102_CONFIG 1
//Config all the sensors
#define CONFIG_SENSORS 0			/*No extra sensors to configure yet*/
#define SCHEDULE_CONFIG I2C1_Request_Job(TMP102_CONFIG);Jobs|=CONFIG_SENSORS/*Just adds directly - job request call starts i2c interrupts off*/

//Function prototypes
void I2C1_Request_Job(uint8_t job_);//Requests a job
void I2C1_Setup_Job(uint8_t job_, volatile uint8_t* data);//Sets up the data pointer for a job
void I2C_Config(void);//configures the hardware
#define Flipbytes(x) x=(((uint16_t)x>>8)&0x00FF)|((((uint16_t)x&0x00FF)<<8)&0xFF00)
#define Flipedbytes(x) (int16_t)(((x>>8)&0x00FF)|(((x&0x00FF)<<8)&0xFF00))
