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

void shutdown_filesystem() {
	char c[]="\r\nLogger turned off\r\n";
	uint8_t a;
	f_write(&FATFS_logfile,c,sizeof(c),&a);		//Write the error to the file
	f_sync(&FATFS_logfile);				//Flush buffers
	f_truncate(&FATFS_logfile);			//Truncate the lenght - fix pre allocation
	f_close(&FATFS_logfile);			//Close any opened file
}
