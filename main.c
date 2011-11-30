#include <string.h>

#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "interrupts.h"
#include "Util/rprintf.h"
#include "Sensors/pressure.h"
#include "usb_lib.h"
#include "Util/USB/hw_config.h"
#include "Util/USB/usb_pwr.h"
#include "Util/fat_fs/inc/diskio.h"
#include "Util/fat_fs/inc/ff.h"
#include "Util/fat_fs/inc/integer.h"
#include "Util/fat_fs/inc/rtc.h"

//Global variables - other files (e.g. hardware interface/drivers) may have their own globals
extern uint16_t MAL_Init (uint8_t lun);			//For the USB filesystem driver
volatile uint8_t file_opened=0;				//So we know to close any opened files before turning off
uint8_t print_string[256];				//For printf data
UINT a;							//File bytes counter
//FatFs filesystem globals go here
FRESULT f_err_code;
static FATFS FATFS_Obj;
FIL FATFS_logfile;

int main(void)
{
	int a=0;
	SystemInit();					//Sets up the clk
	setup_gpio();					//Initialised pins, and detects boot source
	SysTick_Configuration();			//Start up system timer at 100Hz for uSD card functionality
	rtc_init();					//Real time clock initialise - (keeps time unchanged if set)
	ADC_Configuration();
	Usarts_Init();
	calibrate_sensor();				//Calibrate the offset on the diff pressure sensor
	EXTI_Config();
	rprintfInit(__usart_send_char);			//Printf over the bluetooth
	if(USB_SOURCE==bootsource) {
		Set_System();				//This actually just inits the storage layer
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
		while (bDeviceState != CONFIGURED);	//Wait for USB config
		USB_Configured_LED();
	}
	else {
		a=Set_System();				//This actually just inits the storage layer - returns 0 for success
		//a|=init_function();			//Other init functions
		if((f_err_code = f_mount(0, &FATFS_Obj)))Usart_Send_Str((char*)"FatFs mount error\r\n");//This should only error if internal error
		else {					//FATFS initialised ok, try init the card, this also sets up the SPI1
			if((f_err_code=f_open(&FATFS_logfile,"logfile.txt",FA_CREATE_ALWAYS | FA_WRITE))) {//Present
				printf("FatFs drive error %d\r\n",f_err_code);
				if(f_err_code==FR_DISK_ERR || f_err_code==FR_NOT_READY)
					Usart_Send_Str((char*)"No uSD card inserted?\r\n");
			}
			else{				//We have a mounted card
				f_err_code=f_lseek(&FATFS_logfile, PRE_SIZE);// Pre-allocate clusters
				if (f_err_code || f_tell(&FATFS_logfile) != PRE_SIZE)// Check if the file size has been increased correctly
					Usart_Send_Str((char*)"Pre-Allocation error\r\n");
				else {
					if(f_lseek(&FATFS_logfile, 0))//Seek back to start of file to start writing
						Usart_Send_Str((char*)"Seek error\r\n");
					else
						rprintfInit(__str_print_char);//Printf to the logfile
				}
				if(f_err_code)
					f_close(&FATFS_logfile);//Close the already opened file on error
				else file_opened=1;	//So we know to close the file properly on shutdown
			}
		}
		a|=f_err_code;
		if(a) {					//There was an init error
			RED_LED_ON;
			delay();
			shutdown();			//Abort after a single red flash
		}
	}
	while (1) {
		switch_leds_on();
		delay();
		switch_leds_off();
		delay();
		printf("Pressure:%f\r\n",conv_adc_diff());
		if(file_opened) {
			f_write(&FATFS_logfile,print_string,(UINT)strlen(print_string),&a);
			print_string[0]=0x00;		//Set string length to 0
		}
	}
}

/**
  * @brief  Writes a char to logfile
  * @param  Character to write
  * @retval None
  */
void __fat_print_char(char c) {
	f_write(&FATFS_logfile,&c,1,&a);
}

/**
  * @brief  Writes a char to string - use for better logfile performance
  * @param  Character to write
  * @retval None
  */
void __str_print_char(char c) {
	uint8_t a=strlen(print_string)%255;		//Make sure we cant overwrite ram
	print_string[a]=c;				//Append string
	print_string[a+1]=0x00;				//Null terminate
}
