#pragma once
#include "stm32f10x.h"

#include "gpio.h"
#include "pwr.h"

void EXTI_Config(void);
void SysTick_Configuration(void);
void EXTI0_IRQHandler(void);
