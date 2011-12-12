#pragma once
#include "stm32f10x.h"
void calibrate_sensor(void);
float conv_adc_diff(void);
float conv_diff(uint16_t diff);
