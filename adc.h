#include "stm32f10x.h"

#define CRT_PPG_ADC_CHAN 8	//9 is also possible - second PPG channel PORTB0,1

extern uint16_t * ADC1_Convertion_buff;	//malloc this

void ADC_Configuration(void);
uint16_t readADC2(uint8_t channel);
void setADC2(uint8_t channel);
int16_t getADC2(void);
