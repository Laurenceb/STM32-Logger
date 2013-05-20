#include <stdlib.h>
#include "interrupts.h"

volatile uint8_t Button_hold_tim;				//Timer for On/Off/Control button functionality

/**
  * @brief  Configure all interrupts accept on/off pin
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void ISR_Config(void) {
	NVIC_InitTypeDef   NVIC_InitStructure;
	/* Set the Vector Table base location at 0x08000000 */    
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);      
	//First we configure the systick ISR
	/* Configure one bit for preemption priority */   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable and set SYSTICK Interrupt to the fifth priority */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;	//The 100hz timer triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;	//6th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;	//The ADC watchdog triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Low Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;	//7th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//Now we configure the I2C Event ISR
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;	//The I2C1 triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;	//Second to highest group priority
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//Now we configure the I2C Error ISR
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;	//The I2C1 triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	//Highest group priority
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#ifdef BOOTLOADER
	/* Enabling interrupt from USART1 - bluetooth commands, e.g. enter bootloader*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//UAVtalk Rx triggered interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Low pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;	//Third highest group - above the dma
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
#endif
}

/**
  * @brief  Configure on/off pin interrupt
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void EXTI_ONOFF_EN(void) {
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_DeInit();
	/* Connect EXTI0 Line to PA.0 pin - WKUP*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	if(USB_SOURCE==bootsource)				//If we booted from USB, disconnect gives -ive pulse
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	else
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	//The WKUP triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;	//lowest group priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure the DMA interrupt from ADC1 - do this before configuring the ADC+DMA?
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void DMA_ISR_Config(void) {
	NVIC_InitTypeDef   NVIC_InitStructure;
	/* Enable and set DMA1 Interrupt to the sixth priority */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;//The DMA complete/half complete triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Higher pre-emption priority - can nest inside USB/SD
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;	//5th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  This function configures the systick timer to 100hz overflow
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void) {
	RCC_HCLKConfig(RCC_SYSCLK_Div1);			//CLK the periferal - configure the AHB clk
	SysTick_Config(90000);					//SYSTICK at 100Hz - this function also enables the interrupt
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);   //SYSTICK AHB1/8
}

/**
  * @brief  This function handles External line 0 interrupt request.- WKUP ISR
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void EXTI0_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(USB_SOURCE!=bootsource && GET_VBUS_STATE) {	//Interrupt due to USB insertion - reset to usb mode
                        Shutdown_System=USB_INSERTED;		//Request a software reset of the system - USB inserted whilst running
		}
		if(USB_SOURCE==bootsource) {
			if(file_opened) 
				shutdown_filesystem(1,file_opened);//This should not happen
			red_flash();				//Flash red led - provides some debouncing on jack removal
			shutdown();				//Shuts down - only wakes up on power pin i.e. WKUP
		}
		/*Called Code goes here*/
		Button_hold_tim=BUTTON_TURNOFF_TIME;
		RED_LED_ON;					//Red LED is used to indicate successful button press to the user
	}
}

/**
  * @brief  This function handles DMA channel interrupt request.- PPG adc data ISR
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void DMAChannel1_IRQHandler(void) {
	static uint8_t decimation_counter;
	if(DMA_GetITStatus(DMA1_IT_HT1)) {
		DMA_ClearITPendingBit(DMA1_IT_GL1);		//clear all the interrupts
		if(Sensors&PPG_SENSORS)				//PPG enabled
			PPG_LO_Filter(ADC1_Convertion_buff);	//Process lower half
	}
	else if (DMA_GetITStatus(DMA1_IT_TC1)) {
		DMA_ClearITPendingBit(DMA1_IT_GL1);		//clear all the interrupts
		if(Sensors&PPG_SENSORS)				//PPG enabled
			PPG_LO_Filter(&ADC1_Convertion_buff[ADC_BUFF_SIZE/4]);//Transfer complete, process upper half - indexed as 16bit words
	}
	DMA_ClearFlag(DMA1_FLAG_TC1|DMA1_FLAG_HT1);  		//make sure flags are clear
	//Now we process other sensor data - we do this here so as to have all the sensors syncronised with the PPG data
	if(++decimation_counter==PPG_NO_SUBSAMPLES) {		//Each time this is true we will have output some PPG samples
		decimation_counter=0;				//Reset this here
		//Now process each sensor
		if(Sensors&(1<<PRESSURE_HOSE))			//Only pass data once hose is connected
			Add_To_Buffer(*(uint32_t*)(&Reported_Pressure),&Pressures_Buffer);//Pass pressure data via buffer to avoid issues with lag
		if(Sensors&(1<<TEMPERATURE_SENSOR))		//Only pass data is I2C temp sensor is connected
			Add_To_Buffer(*(uint32_t*)(&TMP102_Reported_Temperature),&Temperatures_Buffer);//Pass press data via buffer avoiding lag issue
		if(Sensors&(1<<THERMISTOR_SENSOR))		//Only pass data is I2C temp sensor is connected
			Add_To_Buffer(*(uint32_t*)(&Device_Temperature),&Thermistor_Buffer);//Pass press data via buffer avoiding lag issue
		//More sensors can be added here	
	}
}


/**
  * @brief  This function handles ADC1-2 interrupt requests.- Should only be from the analog watchdog
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void ADC1_2_IRQHandler(void) {
	if(ADC_GetITStatus(ADC2, ADC_IT_AWD))			//Analogue watchdog was triggered
		Shutdown_System=LOW_BATTERY;			//Shutdown to save battery
	ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
	ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);		//None of these should ever happen, but best to be safe
	ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);		//make sure flags are clear
}


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler - runs at 100hz.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void SysTickHandler(void)
{
	static float I,old_pressure;
	static uint16_t Enabled_iterations;			//Note, this is going to break if we spend long periods with +ive pressure set
	static uint32_t Last_Button_Press;			//Holds the timestamp for the previous button press
	static uint8_t System_state_counter;			//Holds the system state counter
	static uint8_t tmpindex;				//Temp sensor decimator
	//FatFS timer function
	disk_timerproc();
	//Incr the system uptime
	Millis+=10;
	if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC)) {		//We have adc2 converted data from the injected channels
		ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);		//Clear the flag
		if(Pressure_Offset)				//Only run the filter when we are sure the sensor is calibrated
			Reported_Pressure=filterloop(conv_diff(ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1)));//convert injected channel 1
		//Now handle the pressure controller
		if(Pressure_control&0x7F) {//If active pressure control is enabled
			//run a PI controller on the air pump motor
			if(Pressure_Setpoint>0 && ( Button_hold_tim>(BUTTON_TURNOFF_TIME-BUTTON_MULTIPRESS_TIMEOUT) || !Button_hold_tim ) ) {
				// A Negative setpoint or prolonged button press forces a dump of air
				float error=Pressure_Setpoint-Reported_Pressure;//Pressure_Setpoint is a global containing the target diff press
				if(Enabled_iterations++>I_HOLDOFF) {
					I+=error*PRESSURE_I_CONST;//Constants defined in main.h
					if(I>PRESSURE_I_LIM)	//Enforce limits
						I=PRESSURE_I_LIM;
					if(I<-PRESSURE_I_LIM)
						I=-PRESSURE_I_LIM;
				}
				int16_t a=PRESSURE_P_CONST*error+I+PRESSURE_D_CONST*(Reported_Pressure-old_pressure);
				if(a>0)				//Make sure we are actually turning the motor on
					Set_Motor((int16_t)a);	//Set the motor gpio dir & pwm duty
			}
			else {
				Enabled_iterations=0;		//Make sure this is reset
				if(abs(Reported_Pressure)>PRESSURE_MARGIN)
					Set_Motor(-1);		//Set a dump to rapidly drop to zero pressure
				else
					Set_Motor(0);
			}
		}			
		else if(!Pressure_control)			//If the most significant bit isnt set
			Set_Motor(0);				//Sets the Rohm motor controller to idle (low current shutdown) state
		//Check the die temperature - not possible on adc1 :-(
		//Device_Temperature=convert_die_temp(ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3));//The on die temperature sensor
		#if BOARD>=3
		if(Sensors&(1<<THERMISTOR_SENSOR))
			Device_Temperature=convert_thermistor_temp(ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3));
		#endif
		//Could process some more sensor data here
		old_pressure=Reported_Pressure;			//Set the old pressure record here for use in the D term
	}
	ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);		//Trigger the injected channel group
	//Read any I2C bus sensors here (100Hz)
	if(Sensors&(1<<TEMPERATURE_SENSOR)) {
		TMP102_Reported_Temperature=GET_TMP_TEMPERATURE;
		if(!tmpindex--) {				//Every 30ms
			tmpindex=3;
			//Jobs|=1<<TMP102_CONFIG;
			I2C1_Request_Job(TMP102_READ);		//Request a TMP102 read if there is one present
			I2C1_Request_Job(TMP102_CONFIG);	//Need to do this to set one shot bit is set high again to start a new single convertion
			//Some sort of i2c error here
		}
	}
	//Now process the control button functions
	if(Button_hold_tim ) {					//If a button press generated timer has been triggered
		if(GET_BUTTON) {				//Button hold turns off the device
			if(!--Button_hold_tim) {
                                Shutdown_System=BUTTON_TURNOFF;//Request turn off of logger after closing any open files
			}
		}
		else {						//Button released - this can only ever run once per press
			RED_LED_OFF;				//Turn off the red LED - used to indicate button press to user
			if(Button_hold_tim<BUTTON_DEBOUNCE) {	//The button has to be held down for longer than the debounce period
				Last_Button_Press=Millis;
				if(++System_state_counter>=SYSTEM_STATES)
					System_state_counter=0;//The system can only have a limited number of states
			}
			Button_hold_tim=0;			//Reset the timer here
		}
	}
	if(Last_Button_Press&&(Millis-Last_Button_Press>BUTTON_MULTIPRESS_TIMEOUT)&&!Button_hold_tim) {//Last press timed out and button is not pressed
		if(!(System_state_Global&0x80))			//The main code has unlocked the global using the bit flag - as it has processed
			System_state_Global=0x80|System_state_counter;//The previous state update
		System_state_counter=0;				//Reset state counter here
		Last_Button_Press=0;				//Reset the last button press timestamp, as the is no button press in play
	}
}

//Included interrupts from ST um0424 mass storage example
#ifndef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_HP_CAN1_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts requests
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void USB_HP_CAN_TX_IRQHandler(void)
{
  CTR_HP();
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void USB_LP_CAN_RX0_IRQHandler(void)
{
  USB_Istr();
}
#endif /* STM32F10X_CL */

#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void SDIO_IRQHandler(void)
{ 
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
  
}
#endif /* STM32F10X_HD | STM32F10X_XL*/

#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : OTG_FS_IRQHandler
* Description    : This function handles USB-On-The-Go FS global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void OTG_FS_IRQHandler(void)
{
  STM32_PCD_OTG_ISR_Handler(); 
}
#endif /* STM32F10X_CL */


__attribute__((externally_visible)) void NMIException(void) {while(1);}
__attribute__((externally_visible)) void HardFaultException(void) {while(1);}
__attribute__((externally_visible)) void MemManageException(void) {while(1);}
__attribute__((externally_visible)) void BusFaultException(void) {while(1);}
__attribute__((externally_visible)) void UsageFaultException(void) {while(1);}
