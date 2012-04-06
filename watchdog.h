//Dactyl project v1.0

#include "stm32f10x.h"
//Independant watchdog functionality

/* Uncomment/Comment depending on your STM32 device. 
   The LSI is internally connected to TIM5 IC4 only on STM32F10x Connectivity 
   line, High-Density Value line, High-Density and XL-Density Devices */
//#define LSI_TIM_MEASURE

/* Macro to reload IWDG counter */
#define  Watchdog_Reset() IWDG_ReloadCounter()

//Function prototypes
#ifdef LSI_TIM_MEASURE
void TIM5_ConfigForLSI(void);
#endif
uint8_t Watchdog_Reset_Detect(void);
void Watchdog_Config(uint16_t timeout_ms);
