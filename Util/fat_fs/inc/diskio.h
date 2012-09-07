/*-----------------------------------------------------------------------
/  Low level disk interface modlue include file
/-----------------------------------------------------------------------*/
#define BOOL bool

#ifndef _DISKIO

#define _READONLY	0	/* 1: Remove write functions */
#define _USE_IOCTL	1	/* 1: Use disk_ioctl fucntion */

#include "integer.h"
#include "stm32f10x.h"

/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */

int assign_drives (int, int);
DSTATUS disk_initialize (BYTE);
DSTATUS disk_status (BYTE);
DRESULT disk_read (BYTE, volatile BYTE*, DWORD, BYTE);
#if	_READONLY == 0
DRESULT disk_write (BYTE, const BYTE*, DWORD, BYTE);
#endif
DRESULT disk_ioctl (BYTE, BYTE, void*);

void wrapup_transaction(void);		/*Laurence added for spi timing improvements*/
void stop_cmd(void);
static BYTE send_cmd (
	BYTE cmd,		/* Command byte */
	DWORD arg		/* Argument */
);
void release_spi (void);
//BOOL rcvr_datablock (
//	volatile BYTE * buff,		/* Data buffer to store received data */
//	UINT btr			/* Byte count (must be multiple of 4) */
//);

/* Disk Status Bits (DSTATUS) */

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */


/* Command code for disk_ioctrl fucntion */

/* Generic command (defined for FatFs) */
#define CTRL_SYNC			0	/* Flush disk cache (for write functions) */
#define GET_SECTOR_COUNT	1	/* Get media size (for only f_mkfs()) */
#define GET_SECTOR_SIZE		2	/* Get sector size (for multiple sector size (_MAX_SS >= 1024)) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (for only f_mkfs()) */
#define CTRL_ERASE_SECTOR	4	/* Force erased a block of sectors (for only _USE_ERASE) */

/* Generic command */
#define CTRL_POWER			5	/* Get/Set power status */
#define CTRL_LOCK			6	/* Lock/Unlock media removal */
#define CTRL_EJECT			7	/* Eject media */

/* MMC/SDC specific ioctl command */
#define MMC_GET_TYPE		10	/* Get card type */
#define MMC_GET_CSD			11	/* Get CSD */
#define MMC_GET_CID			12	/* Get CID */
#define MMC_GET_OCR			13	/* Get OCR */
#define MMC_GET_SDSTAT		14	/* Get SD status */

/* ATA/CF specific ioctl command */
#define ATA_GET_REV			20	/* Get F/W revision */
#define ATA_GET_MODEL		21	/* Get model name */
#define ATA_GET_SN			22	/* Get serial number */

/* NAND specific ioctl command */
#define NAND_FORMAT			30	/* Create physical format */

/* Martin Thomas begin */

/* Card type flags (CardType) */
#define CT_MMC              0x01
#define CT_SD1              0x02
#define CT_SD2              0x04
#define CT_SDC              (CT_SD1|CT_SD2)
#define CT_BLOCK            0x08

#ifndef RAMFUNC
#define RAMFUNC
#endif
RAMFUNC void disk_timerproc (void);

/* Martin Thomas end */

//Macro added by Laurence
#define DMA_Channel_SPI_SD_RX DMA1_Channel4
extern volatile BYTE Sd_Spi_Called_From_USB_MSC;
//End Laurence
//Moved my Laurence
// demo uses a command line option to define this (see Makefile):
#define STM32_SD_USE_DMA
#define MULTI_SPI		/*Allows spi setup to take place here, even if there is no card*/

#ifdef STM32_SD_USE_DMA
// #warning "Information only: using DMA"
#pragma message "*** Using DMA ***"
#endif

/* set to 1 to provide a disk_ioctrl function even if not needed by the FatFs */
#define STM32_SD_DISK_IOCTRL_FORCE      0

// demo uses a command line option to define this (see Makefile):
#if BOARD<3
#define USE_EK_STM32F //this is ported to our hardware - CRT pcb
#else
#define USE_EJ_STM32F //this is ported to our hardware - CRT pcb
#endif
//#define USE_STM32_P103
//#define USE_MINI_STM32

#if defined(USE_EK_STM32F)
 #define CARD_SUPPLY_SWITCHABLE   0		/*we dont have switchable */
 #define GPIO_PWR                 GPIOD
 #define RCC_APB2Periph_GPIO_PWR  RCC_APB2Periph_GPIOD
 #define GPIO_Pin_PWR             GPIO_Pin_10
 #define GPIO_Mode_PWR            GPIO_Mode_Out_OD /* pull-up resistor at power FET */
 #define SOCKET_WP_CONNECTED      0		/*These are defined as 0 as they arent supported on CRT 1.0 pcb*/
 #define SOCKET_CP_CONNECTED      0
 #define SPI_SD                   SPI1
 #define GPIO_CS                  GPIOB
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOB
 #define GPIO_Pin_CS              SD_SEL_PIN
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel2
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel3
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_FLAG_TC2
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_FLAG_TC3
 #define GPIO_SPI_SD              GPIOA
 #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_5
 #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_6
 #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_7
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB2PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB2Periph_SPI1
 /* - for SPI1 and full-speed APB2: 72MHz/(4)=18mhz */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_4/*Note that the ST perif lib defines prescale as F_APB/S_SPI*/

#elif defined(USE_EJ_STM32F)
 #define CARD_SUPPLY_SWITCHABLE   0		/*we dont have switchable */
 #define GPIO_PWR                 GPIOD
 #define RCC_APB2Periph_GPIO_PWR  RCC_APB2Periph_GPIOD
 #define GPIO_Pin_PWR             GPIO_Pin_10
 #define GPIO_Mode_PWR            GPIO_Mode_Out_OD /* pull-up resistor at power FET */
 #define SOCKET_WP_CONNECTED      0		/*These are defined as 0 as they arent supported on CRT 1.0 pcb*/
 #define SOCKET_CP_CONNECTED      0
 #define SPI_SD                   SPI2
 #define GPIO_CS                  GPIOB
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOB
 #define GPIO_Pin_CS              SD_SEL_PIN
 //#define DMA_Channel_SPI_SD_RX    DMA1_Channel4
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel5
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_FLAG_TC4
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_FLAG_TC5
 #define GPIO_SPI_SD              GPIOB
 #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_13
 #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_14
 #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_15
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB1PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB1Periph_SPI2
 /* - for SPI2 and full-speed APB1: 36MHz/(2)=18mhz */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_4//2/*Note that the ST perif lib defines prescale as F_APB/S_SPI*/

#elif defined(USE_STM32_P103)
 // Olimex STM32-P103 not tested!
 #define CARD_SUPPLY_SWITCHABLE   0
 #define SOCKET_WP_CONNECTED      1 /* write-protect socket-switch */
 #define SOCKET_CP_CONNECTED      1 /* card-present socket-switch */
 #define GPIO_WP                  GPIOC
 #define GPIO_CP                  GPIOC
 #define RCC_APBxPeriph_GPIO_WP   RCC_APB2Periph_GPIOC
 #define RCC_APBxPeriph_GPIO_CP   RCC_APB2Periph_GPIOC
 #define GPIO_Pin_WP              GPIO_Pin_6
 #define GPIO_Pin_CP              GPIO_Pin_7
 #define GPIO_Mode_WP             GPIO_Mode_IN_FLOATING /* external resistor */
 #define GPIO_Mode_CP             GPIO_Mode_IN_FLOATING /* external resistor */
 #define SPI_SD                   SPI2
 #define GPIO_CS                  GPIOB
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOB
 #define GPIO_Pin_CS              GPIO_Pin_12
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel4
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel5
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_FLAG_TC4
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_FLAG_TC5
 #define GPIO_SPI_SD              GPIOB
 #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_13
 #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_14
 #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_15
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB1PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB1Periph_SPI2
 /* for SPI2 and full-speed APB1: 36MHz/2 */
 /* !! PRESCALE 4 used here - 2 does not work, maybe because
       of the poor wiring on the HELI_V1 prototype hardware */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_4

#elif defined(USE_MINI_STM32)
 #define CARD_SUPPLY_SWITCHABLE   0
 #define SOCKET_WP_CONNECTED      0
 #define SOCKET_CP_CONNECTED      0
 #define SPI_SD                   SPI1
 #define GPIO_CS                  GPIOB
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOB
 #define GPIO_Pin_CS              GPIO_Pin_6
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel2
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel3
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_FLAG_TC2
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_FLAG_TC3
 #define GPIO_SPI_SD              GPIOA
 #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_5
 #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_6
 #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_7
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB2PeriphClockCmd
 #define RCC_APBPeriph_SPI_SD     RCC_APB2Periph_SPI1
 /* - for SPI1 and full-speed APB2: 72MHz/4 */
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_4

#else
#error "unsupported board"
#endif
//End
#define _DISKIO
#endif
