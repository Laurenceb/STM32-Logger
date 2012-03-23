#include <stdlib.h>
#include "interrupts.h"
#include "main.h"
#include "adc.h"
#include "timer.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "Util/fat_fs/inc/diskio.h"
#include "Util/fat_fs/inc/ff.h"
#include "Sensors/pressure.h"
#include "Sensors/ppg.h"
#include "Sensors/temperature.h"
#include "core_cm3.h"
#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
 #include "stm32_eval_sdio_sd.h"
#endif /* STM32F10X_HD | STM32F10X_XL*/


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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;	//5th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;	//The 100hz timer triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Low Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;	//6th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;	//low group priority
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//Higher pre-emption priority - can nest inside USB/SD
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;	//6th subpriority
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
void EXTI0_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		/*Called Code goes here*/
		delay();					//Debouncing delay
		if(file_opened) {
			char c[]="\r\nLogger turned off\r\n";
			uint8_t a;
			f_write(&FATFS_logfile,c,sizeof(c),&a);	//Write the error to the file
			f_sync(&FATFS_logfile);			//Flush buffers
			f_truncate(&FATFS_logfile);		//Truncate the lenght - fix pre allocation
			f_close(&FATFS_logfile);		//Close any opened file
		}
		if(GET_CHRG_STATE)				//Interrupt due to USB insertion - reset to usb mode
			NVIC_SystemReset();			//Software reset of the system - USB inserted whilst running
		else {
			//red_flash();				//Flash red led
			shutdown();				//Shuts down - only wakes up on power pin i.e. WKUP
		}
	}
}

/**
  * @brief  This function handles DMA channel interrupt request.- PPG adc data ISR
  * @param  None
  * @retval None
  */
void DMAChannel1_IRQHandler(void) {
	if ( DMA_GetITStatus(DMA1_IT_HT1) )			//Half transfer completed
		PPG_LO_Filter(ADC1_Convertion_buff);		//Process lower half
	else
		PPG_LO_Filter(&ADC1_Convertion_buff[ADC_BUFF_SIZE/4]);//Transfer complete, process upper half - indexed as 16bit words
	DMA_ClearFlag(DMA1_FLAG_TC1|DMA1_FLAG_HT1);  		//make sure flags are clear
}

/**
  * @brief  This function handles ADC1-2 interrupt requests.- Should only be from the analogu watchdog
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void) {
	if(ADC_GetITStatus(ADC2, ADC_IT_AWD)) {			//Analogue watchdog was triggered
		if(file_opened) {
			char c[]="\r\nLow Battery\r\n";
			uint8_t a;
			f_write(&FATFS_logfile,c,sizeof(c),&a);	//Write the error to the file
			f_sync(&FATFS_logfile);			//Flush buffers
			f_truncate(&FATFS_logfile);		//Truncate the lenght - fix pre allocation
			f_close(&FATFS_logfile);		//Close any opened file
		}
		red_flash();					//Flash red led
		shutdown();					//Shutdown to save battery
	}
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
void SysTickHandler(void)
{
	static float I,old_pressure;
	static uint16_t Enabled_iterations;			//Note, this is going to break if we spend long periods with +ive pressure set
	//FatFS timer function
	disk_timerproc();
	//Incr the system uptime
	Millis+=10;
	if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC)) {		//We have adc2 converted data from the injected channels
		ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);		//Clear the flag
		if(pressure_offset)				//Only run the filter when we are sure the sensor is calibrated
			reported_pressure=filterloop(conv_diff(ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1)));//convert injected channel 1
		//Now handle the pressure controller
		if(Pressure_control) {//If active pressure control is enabled
			//run a PI controller on the air pump motor
			if(pressure_setpoint>0) {		//A negative setpoint forces a dump of air
				float error=pressure_setpoint-reported_pressure;//pressure_setpoint is a global containing the target diff press
				if(Enabled_iterations++>I_HOLDOFF) {
					I+=error*PRESSURE_I_CONST;//Constants defined in main.h
					if(I>PRESSURE_I_LIM)	//Enforce limits
						I=PRESSURE_I_LIM;
					if(I<-PRESSURE_I_LIM)
						I=-PRESSURE_I_LIM;
				}
				int16_t a=PRESSURE_P_CONST*error+I+PRESSURE_D_CONST*(reported_pressure-old_pressure);
				if(a>0)				//Make sure we are actually turning the motor on
					Set_Motor((int16_t)a);	//Set the motor gpio dir & pwm duty
			}
			else {
				Enabled_iterations=0;		//Make sure this is reset
				if(abs(reported_pressure)>PRESSURE_MARGIN)
					Set_Motor(-1);		//Set a dump to rapidly drop to zero pressure
				else
					Set_Motor(0);
			}
		}			
		else
			Set_Motor(0);				//Sets the Rohm motor controller to idle (low current shutdown) state
		//Check the die temperature - not possible on adc1 :-(
		//Device_Temperature=convert_die_temp(ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3));//The on die temperature sensor
		//Could process some more sensor data here
		old_pressure=reported_pressure;			//Set the old pressure record here for use in the D term
	}
	ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);		//Trigger the injected channel group
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
void USB_HP_CAN_TX_IRQHandler(void)
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
void USB_LP_CAN_RX0_IRQHandler(void)
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
void SDIO_IRQHandler(void)
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
void OTG_FS_IRQHandler(void)
{
  STM32_PCD_OTG_ISR_Handler(); 
}
#endif /* STM32F10X_CL */

