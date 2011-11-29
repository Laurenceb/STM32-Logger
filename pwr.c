#include "pwr.h"

void setuppwr() {
	PWR_DeInit();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//clk to the pwr control
}

void shutdown() {
	PWR_WakeUpPinCmd(ENABLE);			//enable the pin
	PWR_EnterSTANDBYMode();				//only wakes on RTC signals or WKUP pin
}

void disable_pin() {
	PWR_WakeUpPinCmd(DISABLE);			//disable the pin
}
