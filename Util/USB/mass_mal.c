/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : mass_mal.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Medium Access Layer interface
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "platform_config.h"
#ifdef USE_STM3210E_EVAL
 #include "stm32_eval_sdio_sd.h"
#else
 //#include "stm32_eval_spi_sd.h"
#endif /* USE_STM3210E_EVAL */

#ifdef USE_STM3210E_EVAL
 #include "fsmc_nand.h"
 #include "nand_if.h"
#endif /* USE_STM3210E_EVAL */

#include "mass_mal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Mass_Memory_Size[2];
uint32_t Mass_Block_Size[2];
uint32_t Mass_Block_Count[2];
__IO uint32_t Status = 0;
/* Public variables ----------------------------------------------------------*/
volatile uint32_t* Data_Buffer;		/*data buffer for DMA transfers*/
#ifdef USE_STM3210E_EVAL
SD_CardInfo mSDCardInfo;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MAL_Init
* Description    : Initializes the Media on the STM32
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Init(uint8_t lun)
{
  uint16_t status = MAL_OK;

  switch (lun)
  {
    case 0:
      Status = disk_initialize(0);/* Physical drive number (0) */
      if(Status) Status = disk_initialize(0);/*Try again on error*/
      if(!Status && !Data_Buffer) Data_Buffer=(volatile uint32_t*)malloc(MAX_DMA_BUFF_SIZE);/*Allocate the data buffer*/
      break;
#ifdef USE_STM3210E_EVAL
    case 1:
      NAND_Init();
      break;
#endif
    default:
      return MAL_FAIL;
  }
  return status;
}
/*******************************************************************************
* Function Name  : MAL_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, volatile uint8_t * Writebuff, uint32_t Transfer_Length)
{
  switch (lun)
  {
    case 0:/* Physical drive number (0) */
      while(Sd_Spi_Called_From_USB_MSC){;}
      Status = disk_write (0, (volatile uint8_t*)Writebuff, Memory_Offset/512, Transfer_Length/512);//assume integer sectors - 512 bytes
      //Status = SD_WriteBlock((uint8_t*)Writebuff, Memory_Offset, Transfer_Length);
#ifdef USE_STM3210E_EVAL
      if ( Status != SD_OK )
      {
        return MAL_FAIL;
      }      
#endif /* USE_STM3210E_EVAL */      
      break;
#ifdef USE_STM3210E_EVAL
    case 1:
      NAND_Write(Memory_Offset, Writebuff, Transfer_Length);
      break;
#endif /* USE_STM3210E_EVAL */  
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, volatile uint8_t * Readbuff, uint32_t Transfer_Length)
{
  switch (lun)
  {
    case 0: /* Physical drive number (0) */
      Status = disk_read (0, (volatile uint8_t*)Readbuff, Memory_Offset/512, Transfer_Length/512);
      //Status = SD_ReadBlock((uint8_t*)Readbuff, Memory_Offset, Transfer_Length);
#ifdef USE_STM3210E_EVAL      
      if ( Status != SD_OK )
      {
        return MAL_FAIL;
      }
#endif /* USE_STM3210E_EVAL */ 
      if(Status)
	return MAL_FAIL;   
      break;
#ifdef USE_STM3210E_EVAL
    case 1:
      NAND_Read(Memory_Offset, Readbuff, Transfer_Length);
      ;
      break;
#endif
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_GetStatus
* Description    : Get status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_GetStatus (uint8_t lun)
{
#ifdef USE_STM3210E_EVAL
  NAND_IDTypeDef NAND_ID;
  uint32_t DeviceSizeMul = 0, NumberOfBlocks = 0;
#else
  //uint32_t temp_block_mul = 0;
  //SD_CSD SD_csd;
  //uint32_t DeviceSizeMul = 0;
#endif /* USE_STM3210E_EVAL */


  if (lun == 0)
  {
#ifdef USE_STM3210E_EVAL
    if (SD_Init() == SD_OK)
    {
      SD_GetCardInfo(&mSDCardInfo);
      SD_SelectDeselect((uint32_t) (mSDCardInfo.RCA << 16));
      DeviceSizeMul = (mSDCardInfo.SD_csd.DeviceSizeMul + 2);

      if(mSDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
      {
        Mass_Block_Count[0] = (mSDCardInfo.SD_csd.DeviceSize + 1) * 1024;
      }
      else
      {
        NumberOfBlocks  = ((1 << (mSDCardInfo.SD_csd.RdBlockLen)) / 512);
        Mass_Block_Count[0] = ((mSDCardInfo.SD_csd.DeviceSize + 1) * (1 << DeviceSizeMul) << (NumberOfBlocks/2));
      }
      Mass_Block_Size[0]  = 512;

      Status = SD_SelectDeselect((uint32_t) (mSDCardInfo.RCA << 16)); 
      Status = SD_EnableWideBusOperation(SDIO_BusWide_4b); 
      if ( Status != SD_OK )
      {
        return MAL_FAIL;
      }
       
      Status = SD_SetDeviceMode(SD_DMA_MODE);         
      if ( Status != SD_OK )
      {
        return MAL_FAIL;
      } 
     
#else
    //SD_GetCSDRegister(&SD_csd);
    //DeviceSizeMul = SD_csd.DeviceSizeMul + 2;
    //temp_block_mul = (1 << SD_csd.RdBlockLen)/ 512;/* Physical drive number (0) */
    while(Sd_Spi_Called_From_USB_MSC){;}
    if(disk_ioctl (0, GET_SECTOR_COUNT, &Mass_Block_Count[0]))//Sectors are the same as blocks and 512 bytes long?
	return MAL_FAIL;
    //Mass_Block_Count[0] = ((SD_csd.DeviceSize + 1) * (1 << (DeviceSizeMul))) * temp_block_mul;
    Mass_Block_Size[0] = 512;
#endif /* USE_STM3210E_EVAL */
      Mass_Memory_Size[0] = Mass_Block_Count[0] * Mass_Block_Size[0];
	#ifdef CRT
      GREEN_LED_ON;
	#else
      STM_EVAL_LEDOn(LED2);
	#endif
      return MAL_OK;

#ifdef USE_STM3210E_EVAL
    }
#endif /* USE_STM3210E_EVAL */
  }
#ifdef USE_STM3210E_EVAL
  else
  {
    FSMC_NAND_ReadID(&NAND_ID);
    if (NAND_ID.Device_ID != 0 )
    {
      /* only one zone is used */
      Mass_Block_Count[1] = NAND_ZONE_SIZE * NAND_BLOCK_SIZE * NAND_MAX_ZONE ;
      Mass_Block_Size[1]  = NAND_PAGE_SIZE;
      Mass_Memory_Size[1] = (Mass_Block_Count[1] * Mass_Block_Size[1]);
      return MAL_OK;
    }
  }
#endif /* USE_STM3210E_EVAL */
	#ifdef CRT
      GREEN_LED_OFF;
	#else
      STM_EVAL_LEDOff(LED2);
	#endif
  return MAL_FAIL;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
