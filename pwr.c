#include "pwr.h"

/**
  * @brief  Enables the power domain
  * @param  None
  * @retval None
  */
void setuppwr() {
	PWR_DeInit();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//clk to the pwr control
}

/**
  * @brief  Enables the wakeup pin functionality and places device in low power mode
  * @param  None
  * @retval None
  */
void shutdown() {
	PWR_WakeUpPinCmd(ENABLE);			//enable the pin
	PWR_EnterSTANDBYMode();				//only wakes on RTC signals or WKUP pin
}

/**
  * @brief  Disables the wakeup pin functionality
  * @param  None
  * @retval None
  */
void disable_pin() {
	PWR_WakeUpPinCmd(DISABLE);			//disable the pin
}

/**
  * @brief  This function closes any open files, leaving a file footer in the FATFS_logfile
  * @param  uint8_t reason: reason for shutdown, uses enum definitions in main.h, uint8_t file_flags: flag bits for open files
  * @retval None
  */
void shutdown_filesystem(uint8_t reason, uint8_t file_flags) {
	uint8_t c[25] = {};
	if(reason==BUTTON_TURNOFF)
		strncpy(c,"\r\nLogger turned off\r\n",sizeof(c));
	else if(reason==USB_INSERTED)
		strncpy(c,"\r\nUSB cable inserted\r\n",sizeof(c));
	else
		strncpy(c,"\r\nLow Battery\r\n",sizeof(c));
	if(file_flags&0x01) {
		uint32_t a;
		f_write(&FATFS_logfile,c,strlen(c),&a);	//Write the error to the file
		f_sync(&FATFS_logfile);			//Flush buffers
		f_truncate(&FATFS_logfile);		//Truncate the lenght - fix pre allocation
		f_close(&FATFS_logfile);		//Close any opened file
	}
	if(file_flags&0x02)
		wave_terminate(&FATFS_wavfile_accel);	//Close all opened files - terminate wav files correctly
	if(file_flags&0x04)
		wave_terminate(&FATFS_wavfile_gyro);	//Close all opened files - terminate wav files correctly
}
