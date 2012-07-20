/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#ifdef STM32L1XX_MD
 #include "stm32l1xx_it.h"
#else
#ifndef CRT
 #include "stm32f10x_it.h"
#endif
#endif /* STM32L1XX_MD */
 
#include "hw_config.h"
#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
 #include "stm32_eval_sdio_sd.h"
#endif /* STM32F10X_HD | STM32F10X_XL*/
#include "platform_config.h"
#include "mass_mal.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_lib.h"
#include "../../gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
uint16_t Set_System(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */ 
#if defined(USB_USE_EXTERNAL_PULLUP)
  GPIO_InitTypeDef  GPIO_InitStructure;
#endif /* USB_USE_EXTERNAL_PULLUP */
  
#ifdef STM32L1XX_MD 
  /* Enable the SYSCFG module clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif /* STM32L1XX_MD */ 
  
  
#if !defined (USE_STM3210C_EVAL) && !defined (USE_STM32L152_EVAL) && !defined (CRT)
  /* Enable and Disconnect Line GPIO clock */
  USB_Disconnect_Config();
#endif /* USE_STM3210C_EVAL && USE_STM32L152_EVAL */

#if defined(USB_USE_EXTERNAL_PULLUP)
  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);

  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);  
#endif /* USB_USE_EXTERNAL_PULLUP */  
  
  /* MAL configuration */
  return MAL_Config();
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#ifdef STM32L1XX_MD
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  
#elif defined(STM32F10X_CL)
  /* Select USBCLK source */
  RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);

  /* Enable the USB clock */ 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE) ;
#else
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }

}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

 // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

#ifdef STM32L1XX_MD
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#elif defined(STM32F10X_CL) 
  NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#else
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif /* STM32F10X_CL */

  
#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
#endif /* STM32L1XX_MD */
  
}


/*******************************************************************************
* Function Name  : Led_RW_ON
* Description    : Turn ON the Read/Write LEDs.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Led_RW_ON(void)
{
  GREEN_LED_ON;
}

/*******************************************************************************
* Function Name  : Led_RW_OFF
* Description    : Turn off the Read/Write LEDs.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Led_RW_OFF(void)
{
  GREEN_LED_OFF;
}
/*******************************************************************************
* Function Name  : USB_Configured_LED
* Description    : Turn ON the Read/Write LEDs.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Configured_LED(void)
{
  RED_LED_ON;
}

/*******************************************************************************
* Function Name  : USB_NotConfigured_LED
* Description    : Turn off the Read/Write LEDs.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_NotConfigured_LED(void)
{
  RED_LED_OFF;
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#ifdef STM32L1XX_MD
  if (NewState != DISABLE)
  {
    STM32L15_USB_CONNECT;
  }
  else
  {
    STM32L15_USB_DISCONNECT;
  }  

#elif defined(STM32F10X_CL) 
  if (NewState != DISABLE)
  {
    USB_DevConnect();
  }
  else
  {
    USB_DevDisconnect();
  }
  
#elif !defined(CRT)
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

#ifdef STM32L1XX_MD
  Device_Serial0 = *(uint32_t*)(0x1FF80050);
  Device_Serial1 = *(uint32_t*)(0x1FF80054);
  Device_Serial2 = *(uint32_t*)(0x1FF80064);
#else  
  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);
#endif /* STM32L1XX_MD */

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &MASS_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &MASS_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : MAL_Config
* Description    : MAL_layer configuration
* Input          : None.
* Return         : None.
*******************************************************************************/
uint16_t MAL_Config(void)
{
  return MAL_Init(0);

#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
  /* Enable the FSMC Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
  MAL_Init(1);
#endif /* STM32F10X_HD | STM32F10X_XL */
}

#if defined (USE_STM3210B_EVAL) || defined (USE_STM3210E_EVAL)
/*******************************************************************************
* Function Name  : USB_Disconnect_Config
* Description    : Disconnect pin configuration
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Disconnect_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* USB_DISCONNECT_PIN used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
}
#endif /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */

#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_OTG_BSP_uDelay.
* Description    : provide delay.
* Input          : Delay in micro seconds.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_OTG_BSP_uDelay (const uint32_t usec)
{
  RCC_ClocksTypeDef  RCC_Clocks;  

  /* Configure HCLK clock as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  
  RCC_GetClocksFreq(&RCC_Clocks);
  
  SysTick_Config(usec * (RCC_Clocks.HCLK_Frequency / 1000000));  
  
  SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk ;
  
  while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}
#endif
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
