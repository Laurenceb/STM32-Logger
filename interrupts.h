#pragma once
#include "stm32f10x.h"

#include "gpio.h"
#include "pwr.h"

void ISR_Config(void);
void EXTI_ONOFF_EN(void);
void SysTick_Configuration(void);
void EXTI0_IRQHandler(void);
void DMA_ISR_Config(void);
