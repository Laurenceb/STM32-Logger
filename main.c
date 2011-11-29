/* missing type */
#include "stm32f10x.h"

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

extern uint16_t MAL_Init (uint8_t lun);			//For the USB filesystem driver

int main(void)
{
	int a;
	SystemInit();
	SysTick_Configuration();			//Start up system timer at 100Hz for uSD card functionality
	setup_gpio();
	ADC_Configuration();
	Usarts_Init();
	calibrate_sensor();
	EXTI_Config();
	rprintfInit(__usart_send_char);
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
		if(a) {					//Theres was an init error
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
	}
}
