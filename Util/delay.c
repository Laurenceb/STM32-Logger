//Dactyl project v1.0
#include "delay.h"
/**
  * @brief  A simple delay loop
  * @param  Number of loops
  * @retval None
  * Note this is compensated for the sysclk speed
  */
void Delay(volatile uint32_t delay) {
	delay*=SystemFrequency/(1000000*9);
	for(; delay;){--delay;}
}
