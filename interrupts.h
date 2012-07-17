#pragma once
#include "stm32f10x.h"
#include "main.h"
#include "adc.h"
#include "timer.h"
#include "i2c_int.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "Util/fat_fs/inc/diskio.h"
#include "Util/fat_fs/inc/ff.h"
#include "Sensors/pressure.h"
#include "Sensors/ppg.h"
#include "Sensors/temperature.h"
#include "core_cm3.h"
#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
 #include "stm32_eval_sdio_sd.h"
#endif /* STM32F10X_HD | STM32F10X_XL*/
#include "gpio.h"
#include "pwr.h"

//These times are all in units of 10ms
#define BUTTON_DEBOUNCE 		(80-5)
#define BUTTON_MULTIPRESS_TIMEOUT	45
#define BUTTON_TURNOFF_TIME		80

void ISR_Config(void);
void EXTI_ONOFF_EN(void);
void SysTick_Configuration(void);
void EXTI0_IRQHandler(void);
void DMA_ISR_Config(void);
void DMA_ISR_Config_SPI2(void);
