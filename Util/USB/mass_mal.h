/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : mass_mal.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Header for mass_mal.c file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MASS_MAL_H
#define __MASS_MAL_H

/* Includes ------------------------------------------------------------------*/
#ifndef CRT
 #include "stm32_eval.h"
#else
 #include "../../gpio.h"
 #include "../fat_fs/inc/diskio.h"
 #include "../../interrupts.h"
#endif
/* Exported types ------------------------------------------------------------*/
extern volatile uint32_t* Data_Buffer;		/*data buffer for DMA transfers*/
/* Exported constants --------------------------------------------------------*/
#define MAL_OK   0
#define MAL_FAIL 1
#define MAX_LUN  1

#define MAX_DMA_BUFF_SIZE 5120			/*10 sector buffer speeds write*/

/* Exported macro ------------------------------------------------------------*/
#define MAL_TRANSFER_INDEX DMA_GetCurrDataCounter(DMA_Channel_SPI_SD_RX)
/* Exported functions ------------------------------------------------------- */

uint16_t MAL_Init (uint8_t lun);
uint16_t MAL_GetStatus (uint8_t lun);
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, volatile uint8_t * Readbuff, uint32_t Transfer_Length);
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, volatile uint8_t * Writebuff, uint32_t Transfer_Length);
#endif /* __MASS_MAL_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
