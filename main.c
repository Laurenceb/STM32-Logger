#include <string.h>
#include "stm32f10x.h"
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "interrupts.h"
#include "timer.h"
#include "watchdog.h"
#include "Util/rprintf.h"
#include "Util/delay.h"
#include "Sensors/pressure.h"
#include "Sensors/ppg.h"
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
volatile buff_type Buff[PPG_CHANNELS];			//Shared with ISR
volatile uint8_t Pressure_control;			//Enables the pressure control PI
volatile float pressure_setpoint;			//Target differential pressure for the pump
volatile float reported_pressure;			//Pressure as measured by the sensor
volatile uint32_t Millis;				//System uptime (rollover after 50 days)
volatile float Device_Temperature;			//Die temperature sensor converted to centigrade
volatile uint8_t System_state_Global;			//Stores the system state, controlled by the button, most significant bit is a flag
volatile uint8_t Sensors;				//Global holding a mask of the sensors found by automatic sensor discovery
//Sensor buffers to pass data back to logger
volatile buff_type Temperatures_Buffer;			//Data from temperature sensor
volatile buff_type Pressures_Buffer;
//FatFs filesystem globals go here
FRESULT f_err_code;
static FATFS FATFS_Obj;
FIL FATFS_logfile;
FILINFO FATFS_info;


int main(void)
{
	uint8_t a=0;
	uint32_t ppg[2];				//two PPG channels
	uint32_t data_counter;				//used as data timestamp
	float sensor_data;				//used for handling data passed back from sensors
	RTC_t RTC_time;
	SystemInit();					//Sets up the clk
	setup_gpio();					//Initialised pins, and detects boot source
	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);	//Watchdog stopped during JTAG halt
	Watchdog_Config(WATCHDOG_TIMEOUT);		//Set the watchdog
	SysTick_Configuration();			//Start up system timer at 100Hz for uSD card functionality
	rtc_init();					//Real time clock initialise - (keeps time unchanged if set)
	Usarts_Init();
	setup_pwm();					//Enable the PWM outputs on all three channels
	ISR_Config();
	rprintfInit(__usart_send_char);			//Printf over the bluetooth
	if(USB_SOURCE==bootsource) {
		Set_System();				//This actually just inits the storage layer
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
		while (bDeviceState != CONFIGURED) {	//Wait for USB config - timeout causes shutdown
			if(Millis>10000 || !GET_CHRG_STATE)//No USB cable - shutdown (Charger pin will be set to open drain, cant be disabled without usb)
				shutdown();
			Watchdog_Reset();		//Reset watchdog here, if we are stalled here the Millis timeout should catch us
		}
		USB_Configured_LED();
		EXTI_ONOFF_EN();			//Enable the off interrupt - allow some time for debouncing
		while(1) {				//If running off USB (mounted as mass storage), stay in this loop - dont turn on anything
			if(Millis%1000>500)		//1Hz on/off flashing
				switch_leds_on();	//Flash the LED(s)
			else
				switch_leds_off();
			Watchdog_Reset();
		}
	}
	else {
		if(!GET_PWR_STATE)			//Check here to make sure the power button is still pressed, if not, sleep
			shutdown();			//This means a glitch on the supply line, or a power glitch results in sleep
		a=Set_System();				//This actually just inits the storage layer - returns 0 for success
		//a|=init_function();			//Other init functions
		if((f_err_code = f_mount(0, &FATFS_Obj)))Usart_Send_Str((char*)"FatFs mount error\r\n");//This should only error if internal error
		else {					//FATFS initialised ok, try init the card, this also sets up the SPI1
			if(!f_open(&FATFS_logfile,"time.txt",FA_OPEN_EXISTING | FA_READ | FA_WRITE)) {//Try and open a time file to get the system time
				if(!f_stat((const TCHAR *)"time.txt",&FATFS_info)) {//Get file info
					if(!FATFS_info.fsize) {//Empty file
						RTC_time.year=(FATFS_info.fdate>>9)+1980;//populate the time struct (FAT start==1980, RTC.year==0)
						RTC_time.month=(FATFS_info.fdate>>5)&0x000F;
						RTC_time.mday=FATFS_info.fdate&0x001F;
						RTC_time.hour=(FATFS_info.ftime>>11)&0x001F;
						RTC_time.min=(FATFS_info.ftime>>5)&0x003F;
						RTC_time.sec=(FATFS_info.ftime<<1)&0x003E;
						rtc_settime(&RTC_time);
						rprintfInit(__fat_print_char);//printf to the open file
						printf("RTC set to %d/%d/%d %d:%d:%d\n",RTC_time.mday,RTC_time.month,RTC_time.year,\
						RTC_time.hour,RTC_time.min,RTC_time.sec);
					}				
				}
				f_close(&FATFS_logfile);//Close the time.txt file
			}
			if((f_err_code=f_open(&FATFS_logfile,"logfile.txt",FA_CREATE_ALWAYS | FA_WRITE))) {//Present
				printf("FatFs drive error %d\r\n",f_err_code);
				if(f_err_code==FR_DISK_ERR || f_err_code==FR_NOT_READY)
					Usart_Send_Str((char*)"No uSD card inserted?\r\n");
			}
			else {				//We have a mounted card
				f_err_code=f_lseek(&FATFS_logfile, PRE_SIZE);// Pre-allocate clusters
				if (f_err_code || f_tell(&FATFS_logfile) != PRE_SIZE)// Check if the file size has been increased correctly
					Usart_Send_Str((char*)"Pre-Allocation error\r\n");
				else {
					if((f_err_code=f_lseek(&FATFS_logfile, 0)))//Seek back to start of file to start writing
						Usart_Send_Str((char*)"Seek error\r\n");
					else
						rprintfInit(__str_print_char);//Printf to the logfile
				}
				if(f_err_code)
					f_close(&FATFS_logfile);//Close the already opened file on error
				else
					file_opened=1;	//So we know to close the file properly on shutdown
			}
		}
		a|=f_err_code;
		if(a) {					//There was an init error
			RED_LED_ON;
			Delay(400000);
			shutdown();			//Abort after a single red flash
		}
		init_buffer(&(Buff[0]),PPG_BUFFER_SIZE);//Enough for ~0.25S of data
		init_buffer(&(Buff[1]),PPG_BUFFER_SIZE);
	}
	Delay(100000);					//Sensor+inst amplifier takes about 100ms to stabilise after power on
	ADC_Configuration();				//We leave this a bit later to allow stabilisation
	calibrate_sensor();				//Calibrate the offset on the diff pressure sensor
	EXTI_ONOFF_EN();				//Enable the off interrupt - allow some time for debouncing
	I2C_Config();					//Setup the I2C bus
	Sensors=detect_sensors();			//Search for connected sensors
	Pressure_control=Sensors&PRESSURE_HOSE;		//Enable active pressure control if a hose is connected
	pressure_setpoint=0;				//Not applied pressure, should cause motor and solenoid to go to idle state
	PPG_Automatic_Brightness_Control();		//Run the automatic brightness setting on power on
	rtc_gettime(&RTC_time);				//Get the RTC time and put a timestamp on the start of the file
	printf("%d-%d-%dT%d:%d:%d\n",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);//ISO 8601 timestamp header
	if(file_opened) {
		f_puts(print_string,&FATFS_logfile);
		print_string[0]=0x00;			//Set string length to 0
	}
	Millis=0;					//Reset system uptime, we have 50 days before overflow
	while (1) {
		Watchdog_Reset();			//Reset the watchdog each main loop iteration
		while(!bytes_in_buff(&(Buff[0])));	//Wait for some PPG data
		Get_From_Buffer(&(ppg[0]),&(Buff[0]));	//Retrive one sample of PPG
		Get_From_Buffer(&(ppg[1]),&(Buff[1]));	
		printf("%3f,%lu,%lu",(float)(data_counter++)/PPG_SAMPLE_RATE,ppg[0],ppg[1]);//Print data after a time stamp (not Millis)
		if(Sensors&(1<<PRESSURE_HOSE)) {	//Air hose connected
			do {
				Get_From_Buffer(&sensor_data,&Pressures_Buffer);
			} while(bytes_in_buff(&Pressures_Buffer));//The aquisition will often be running faster than this loop, so dump the unused data
			printf(",%2f",sensor_data);	//print the retreived data
		}
		if(Sensors&(1<<TEMPERATURE_SENSOR)) {	//If there is a temperature sensor present
			do {
				Get_From_Buffer(&sensor_data,&Temperatures_Buffer);
			} while(bytes_in_buff(&Temperatures_Buffer));//The aquisition will often be running faster than this loop, so dump the unused data
			printf(",%2f",sensor_data);	//print the retreived data
		}
		//Other sensors etc can go here
		printf("\n");				//Terminating newline
		if(file_opened) {
			f_puts(print_string,&FATFS_logfile);
			print_string[0]=0x00;		//Set string length to 0
		}
		if(Millis%1000>500)			//1Hz on/off flashing
			switch_leds_on();		//Flash the LED(s)
		else
			switch_leds_off();
		if(Millis%15000>4000)			//15 second cycle of pressure control - 11s dump, 4s pump to 3psi
			pressure_setpoint=-1;
		else
			pressure_setpoint=3;
		if(System_state_Global&0x80) {		//A "control" button press
			System_state_Global&=~0x80;	//Wipe the flag bit to show this has been processed
			PPG_Automatic_Brightness_Control();//At the moment this is the only function implimented
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

/**
  * @brief  Detects which sensors are plugged in, inits buffers for attached peripheral sensors
  * @param  None
  * @retval Bitmask of detected sensors
  */
uint8_t detect_sensors(void) {
	uint32_t millis=Millis;				//Store the time on entry
	uint8_t sensors=0;
	SCHEDULE_CONFIG;				//Run the I2C devices config
	//Detect if there is an air hose connected
	Pressure_control|=0x80;				//Set msb - indicates motor is free to run
	Set_Motor((int16_t)(MAX_DUTY)/2);		//Set the motor to 50% max duty cycle
	while(Millis<(millis+300)) {			//Wait 300ms
		if(reported_pressure>PRESSURE_MARGIN) {	//We got some sane pressure increase
			sensors|=(1<<PRESSURE_HOSE);
			init_buffer(&Pressures_Buffer,TMP102_BUFFER_SIZE);//reuse the TMP102 buffer size - as we want the same amount of buffering
			Pressure_control=0;
			break;				//Exit loop at this point
		}
	}
	Pressure_control=0;
	Set_Motor((int16_t)0);				//Set the motor and solenoid off
	//Detect if there is a temperature sensor connected
	if(Completed_Jobs&(1<<TMP102_CONFIG))
		sensors|=(1<<TEMPERATURE_SENSOR);	//The I2C job completion means the sensor must be working
	init_buffer(&Temperatures_Buffer,TMP102_BUFFER_SIZE);
	//Other sensors, e.g. Temperature sensor/sensors on the I2C bus go here
	return sensors;
}
