#pragma once
#include "stm32f10x.h"

extern volatile float Pressure_Offset;
extern volatile float Reported_Pressure;			//Pressure as measured by the sensor

void calibrate_sensor(void);
float conv_adc_diff(void);
float conv_diff(uint16_t diff);
float filterloop(float input);
