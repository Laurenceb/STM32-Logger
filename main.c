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


//newlib reent context
    struct _reent my_reent;
     


//Global variables - other files (e.g. hardware interface/drivers) may have their own globals
extern uint16_t MAL_Init (uint8_t lun);			//For the USB filesystem driver
volatile uint8_t file_opened=0;				//So we know to close any opened files before turning off
uint8_t print_string[256];				//For printf data
UINT a;							//File bytes counter
volatile buff_type Buff[PPG_CHANNELS];			//Shared with ISR
volatile uint8_t Pressure_control;			//Enables the pressure control PI
volatile float Pressure_Setpoint;			//Target differential pressure for the pump
volatile uint32_t Millis;				//System uptime (rollover after 50 days)
volatile uint8_t System_state_Global;			//Stores the system state, controlled by the button, most significant bit is a flag
volatile uint8_t Sensors;				//Global holding a mask of the sensors found by automatic sensor discovery
volatile uint8_t Shutdown_System;			//Used to order a system shutdown to sleep mode
//Sensor buffers to pass data back to logger
volatile buff_type Temperatures_Buffer;			//Data from temperature sensor
volatile buff_type Thermistor_Buffer;	
volatile buff_type Pressures_Buffer;
//FatFs filesystem globals go here
FRESULT f_err_code;
static FATFS FATFS_Obj;
FIL FATFS_logfile;
FILINFO FATFS_info;
//volatile int bar[3] __attribute__ ((section (".noinit"))) ;//= 0xaa

int main(void)
{
	uint32_t ppg;					//PPG channel
	uint32_t data_counter=0;			//used as data timestamp
	uint8_t system_state=0;				//used to track button press functionality
	float sensor_data;				//used for handling data passed back from sensors
	RTC_t RTC_time;
        _REENT_INIT_PTR(&my_reent);
        _impure_ptr = &my_reent;
	SystemInit();					//Sets up the clk
	setup_gpio();					//Initialised pins, and detects boot source
	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);	//Watchdog stopped during JTAG halt
	if(RCC->CSR&RCC_CSR_IWDGRSTF) {			//Watchdog reset, turn off
		RCC->CSR|=RCC_CSR_RMVF;			//Reset the reset flags
		shutdown();
	}
	SysTick_Configuration();			//Start up system timer at 100Hz for uSD card functionality
	Watchdog_Config(WATCHDOG_TIMEOUT);		//Set the watchdog
	Watchdog_Reset();				//Reset watchdog as soon as possible incase it is still running at power on
	rtc_init();					//Real time clock initialise - (keeps time unchanged if set)
	Usarts_Init();
	ISR_Config();
	rprintfInit(__usart_send_char);			//Printf over the bluetooth
	if(USB_SOURCE==bootsource) {
		Set_System();				//This actually just inits the storage layer
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
		uint32_t nojack=0x000FFFFF;		//Countdown timer - a few hundered ms of 0v on jack detect forces a shutdown
		while (bDeviceState != CONFIGURED) {	//Wait for USB config - timeout causes shutdown
			if((Millis>10000 && bDeviceState == UNCONNECTED)|| !nojack)	//No USB cable - shutdown (Charger pin will be set to open drain, cant be disabled without usb)
				shutdown();
			if(GET_VBUS_STATE)		//Jack detect resets the countdown
				nojack=0x0FFFFF;
			nojack--;
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
			__WFI();			//Sleep mode
		}
	}
	if(!GET_PWR_STATE)				//Check here to make sure the power button is still pressed, if not, sleep
		shutdown();				//This means a glitch on the supply line, or a power glitch results in sleep
	for(uint8_t n=0;n<PPG_CHANNELS;n++)
		init_buffer(&(Buff[n]),PPG_BUFFER_SIZE);//Enough for ~0.25S of data
	setup_pwm();					//Enable the PWM outputs on all three channels
	Delay(100000);					//Sensor+inst amplifier takes about 100ms to stabilise after power on
	ADC_Configuration();				//We leave this a bit later to allow stabilisation
	calibrate_sensor();				//Calibrate the offset on the diff pressure sensor
	EXTI_ONOFF_EN();				//Enable the off interrupt - allow some time for debouncing
	I2C_Config();					//Setup the I2C bus
	uint8_t sensors_=detect_sensors();		//Search for connected sensors
	uint8_t Oversaturation=0;			//Use this to detect oversaturating adc input
	sensor_data=GET_BATTERY_VOLTAGE;		//Have to flush adc for some reason
	Delay(10000);
	if(!(sensors_&~(1<<PRESSURE_HOSE))||GET_BATTERY_VOLTAGE<BATTERY_STARTUP_LIMIT) {//We will have to turn off
		Watchdog_Reset();			//LED flashing takes a while
		if(abs(Reported_Pressure)>PRESSURE_MARGIN)
			Set_Motor(-1);			//If the is air backpressure, dump to rapidly drop to zero pressure before turnoff
		if(file_opened)
			f_close(&FATFS_logfile);	//be sure to terminate file neatly
		red_flash();
		Delay(400000);
		red_flash();				//Two flashes means battery abort -----------------ABORT 2
		if(sensors_&~(1<<PRESSURE_HOSE))
			shutdown();
		Delay(400000);
		red_flash();				//Three flashes means no sensors abort ------------ABORT 3
		shutdown();
	}
	if((f_err_code = f_mount(0, &FATFS_Obj)))Usart_Send_Str((char*)"FatFs mount error\r\n");//This should only error if internal error
	else {						//FATFS initialised ok, try init the card, this also sets up the SPI1
		if(!f_open(&FATFS_logfile,"time.txt",FA_OPEN_EXISTING | FA_READ | FA_WRITE)) {//Try and open a time file to get the system time
			if(!f_stat((const TCHAR *)"time.txt",&FATFS_info)) {//Get file info
				if(!FATFS_info.fsize) {	//Empty file
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
			f_close(&FATFS_logfile);	//Close the time.txt file
		}
#ifndef SINGLE_LOGFILE
		rtc_gettime(&RTC_time);			//Get the RTC time and put a timestamp on the start of the file
		rprintfInit(__str_print_char);		//Print to the string
		printf("%02d-%02d-%02dT%02d-%02d-%02d.csv",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);//Timestamp name
		rprintfInit(__usart_send_char);		//Printf over the bluetooth
#endif
		if((f_err_code=f_open(&FATFS_logfile,LOGFILE_NAME,FA_CREATE_ALWAYS | FA_WRITE))) {//Present
			printf("FatFs drive error %d\r\n",f_err_code);
			if(f_err_code==FR_DISK_ERR || f_err_code==FR_NOT_READY)
				Usart_Send_Str((char*)"No uSD card inserted?\r\n");
		}
		else {					//We have a mounted card
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
				file_opened=1;		//So we know to close the file properly on shutdown
		}
	}
	if(f_err_code) {				//There was an init error
		red_flash();
		shutdown();				//Abort after a single red flash ------------------ABORT 1
	}
	Watchdog_Reset();				//Card Init can take a second or two
	Pressure_control=sensors_&(1<<PRESSURE_HOSE);	//Enable active pressure control if a hose is connected
	Pressure_Setpoint=0;				//Not applied pressure, should cause motor and solenoid to go to idle state
	rtc_gettime(&RTC_time);				//Get the RTC time and put a timestamp on the start of the file
	print_string[0]=0x00;				//Set string length to 0
	printf("%02d-%02d-%02dT%02d:%02d:%02d\n",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);//ISO 8601 timestamp header
	printf("Battery: %3fV\n",GET_BATTERY_VOLTAGE);	//Get the battery voltage using blocking regular conversion and print
	printf("Time");					//Print out the sensors that are present in the CSV file
	if(sensors_&1<<PPG_SENSOR_ZERO)
		printf(", PPG(0)");
	if(sensors_&1<<PPG_SENSOR_ONE)
		printf(", PPG(1)");
	if(sensors_&1<<PPG_SENSOR_TWO)
		printf(", PPG(2)");
	if(sensors_&1<<PRESSURE_HOSE)
		printf(", Pressure");
	if(sensors_&1<<TEMPERATURE_SENSOR)
		printf(", I2C temp sensor");
	if(sensors_&1<<THERMISTOR_SENSOR)
		printf(", Temperature sensor");
	printf(", Button press\r\n");
	if(file_opened) {
		f_puts(print_string,&FATFS_logfile);
		print_string[0]=0x00;			//Set string length to 0
	}
	for(uint8_t n=0;n<PPG_CHANNELS;n++)
		Empty_Buffer(&Buff[n]);			//Empty all the PPG buffers to sync all the data - buffers will have some data from autobrightness
	Millis=0;					//Reset system uptime, we have 50 days before overflow
	Sensors|=sensors_;				//Set the global sensors variable here to mark the detected sensors as present
	while (1) {
		Watchdog_Reset();			//Reset the watchdog each main loop iteration
		while(!bytes_in_buff(&(Buff[0])))
			__WFI();			//Wait for some PPG data
		printf("%3f",(float)(data_counter++)/PPG_SAMPLE_RATE);//The time since PPG collection started
		for(uint8_t n=0;n<PPG_CHANNELS;n++) {	//Loop through the PPG channels
			Get_From_Buffer(&ppg,&(Buff[n]));//Retrive one sample of PPG
			if(Sensors&(1<<(PPG_SENSOR_ZERO+n)))//If sensor is present
				printf(",%lu",ppg);	//Print data after a time stamp (not Millis)
		}
		if(Sensors&(1<<PRESSURE_HOSE)) {	//Air hose connected
			Get_From_Buffer(&sensor_data,&Pressures_Buffer);//This is syncronised with PPG data in PPG ISR using a global defined in the sensor header
			printf(",%2f",sensor_data);	//print the retreived data
		}
		if(Sensors&(1<<TEMPERATURE_SENSOR)) {	//If there is a temperature sensor present
			Get_From_Buffer(&sensor_data,&Temperatures_Buffer);
			printf(",%2f",sensor_data);	//print the retreived data
		}
		if(Sensors&(1<<THERMISTOR_SENSOR)) {	//If there is a thermistor temperature sensor present
			Get_From_Buffer(&sensor_data,&Thermistor_Buffer);
			printf(",%2f",sensor_data);	//print the retreived data
		}
		//Other sensors etc can go here
		//Button multipress status
		if(System_state_Global&0x80) {		//A "control" button press
			system_state=System_state_Global&~0x80;//Copy to local variable
			if(system_state==1)		//A single button press
				PPG_Automatic_Brightness_Control();//At the moment this is the only function implimented
			System_state_Global&=~0x80;	//Wipe the flag bit to show this has been processed
		}
		//Check the level to see if we are saturating
		{uint8_t m=0;
		for( uint8_t n=0;n<PPG_CHANNELS;n++ ) {	//Loop through the PPG channels
			if( Last_PPG_Values[n]>((TARGET_ADC*5)/6) )
				m=1;
		}
		if(m)		
			Oversaturation++;
		else
			Oversaturation=m;
		if( Oversaturation>(uint8_t)(PPG_SAMPLE_RATE*0.25) ) {
			PPG_Automatic_Brightness_Control();//Fix the brightness
			Oversaturation=0;
			system_state=255;		//Marker to show we adjusted automatically
		}
		}
		printf(",%d\n",system_state);		//Terminating newline
		system_state=0;				//Reset this
		if(file_opened  & 0x01) {
			f_puts(print_string,&FATFS_logfile);
			print_string[0]=0x00;		//Set string length to 0
		}
		//Deal with file size - may need to preallocate some more
		if(f_size(&FATFS_logfile)-f_tell(&FATFS_logfile)<(PRE_SIZE/2)) {//More than half way through the pre-allocated area
			DWORD size=f_tell(&FATFS_logfile);
			f_lseek(&FATFS_logfile, f_size(&FATFS_logfile)+PRE_SIZE);//preallocate another PRE_SIZE
			f_lseek(&FATFS_logfile, size);	//Seek back to where we were before
		}
		if(Millis%1000>500)			//1Hz on/off flashing
			switch_leds_on();		//Flash the LED(s)
		else
			switch_leds_off();
                if(Shutdown_System) {			//A system shutdown has been requested
			if(file_opened)
				shutdown_filesystem(Shutdown_System, file_opened);
			if(Shutdown_System==USB_INSERTED)
				NVIC_SystemReset();	//Software reset of the system - USB inserted whilst running
			else {
				if(Shutdown_System==LOW_BATTERY)
					red_flash();	//Used to indicate an error condition before turnoff
				shutdown();		//Puts us into sleep mode
			}
		}
		//Pressure control is here
		if(Millis%20000>4000)			//20 second cycle of pressure control - 16s dump, 4s pump to 3psi
			Pressure_Setpoint=-1;
		else
			Pressure_Setpoint=2.5;		//2.5PSI setpoint
	}
}

/**
  * @brief  Writes a char to logfile
  * @param  Character to write
  * @retval None
  */
void __fat_print_char(char c) {
	f_write(&FATFS_logfile,&c,(UINT)1,&a);
}

/**
  * @brief  Writes a char to string - use for better logfile performance
  * @param  Character to write
  * @retval None
  */
void __str_print_char(char c) {
	uint8_t indx=strlen(print_string)%255;		//Make sure we cant overwrite ram
	print_string[indx]=c;				//Append string
	print_string[indx+1]=0x00;			//Null terminate
	__usart_send_char(c);				//Send to the bluetooth as well
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
	Set_Motor((int16_t)(MAX_DUTY*3)/4);		//Set the motor to 75% max duty cycle
	while(Millis<(millis+500)) {			//Wait 500ms
		if(Reported_Pressure>(PRESSURE_MARGIN*4)) {//We got some sane pressure increase
			sensors|=(1<<PRESSURE_HOSE);
			init_buffer(&Pressures_Buffer,TMP102_BUFFER_SIZE);//reuse the TMP102 buffer size - as we want the same amount of buffering
			break;				//Exit loop at this point
		}
	}
	Pressure_control&=~0x80;			//Clear the Pressure_control msb so that motor is disabled in control off mode
	Set_Motor((int16_t)0);				//Set the motor and solenoid off
	//Detect if there is a temperature sensor connected
	if(Completed_Jobs&(1<<TMP102_CONFIG)) {
		sensors|=(1<<TEMPERATURE_SENSOR);	//The I2C job completion means the sensor must be working
		init_buffer(&Temperatures_Buffer,TMP102_BUFFER_SIZE);
	}
	#if BOARD>=3					/*Only version 3 and newer boards can connect directly to a thermistor*/
	Device_Temperature=convert_thermistor_temp(ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3));//Convert the thermistor ADC inject channel
	if(Device_Temperature<50 && Device_Temperature>-10) {//Sanity check to see if there is something connected
		sensors|=(1<<THERMISTOR_SENSOR);	//if so mark sensor as present
		init_buffer(&Thermistor_Buffer,TMP102_BUFFER_SIZE);
	}
	#endif
	//Other sensors, e.g. Temperature sensor/sensors on the I2C bus go here
	//PPG is detectable
	Sensors|=PPG_SENSORS;				//Set this bit so that the PPG decoder start running
	while(!bytes_in_buff(&(Buff[0])));		//Wait for some PPG data for the auto brightness to work with
	PPG_Automatic_Brightness_Control();		//Run the automatic brightness setting on power on
	if(Get_PWM_0()<=PPG_DETECTION_LIMIT)
		sensors|=(1<<PPG_SENSOR_ZERO);
	if(Get_PWM_1()<=PPG_DETECTION_LIMIT)
		sensors|=(1<<PPG_SENSOR_ONE);
	if(Get_PWM_2()<=PPG_DETECTION_LIMIT)
		sensors|=(1<<PPG_SENSOR_TWO);
	Sensors&=~PPG_SENSORS;				//Make sure this is not set
	return sensors;
}
