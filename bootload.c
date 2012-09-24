#include "stm32f10x.h"
#include "core_cm3.h"
#include "main.h"
#include "ff.h"

void BootLoader(void) {
	void (*SysMemBootJump)(void) = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
	__set_PRIMASK(1);
	RCC_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	__set_MSP(0x20001000);
	SysMemBootJump();
}

__attribute__((externally_visible)) void USART1_IRQHandler(void) {
	static uint8_t dat[4];
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		//Clear pending bit and read the data.
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		*(uint32_t*)dat<<=8;		//Works on little endian architectures
		dat[0]=USART_ReceiveData(USART1);//Send "btld" to enter bootloader
		if(dat[3]=='b' && dat[2]=='t' && dat[1]=='l' && dat[0]=='d') {
			if(file_opened) {
				char c[]="\r\nFirmware update\r\n";
				uint8_t a;
				f_write(&FATFS_logfile,c,sizeof(c),&a);	//Write the error to the file
				f_sync(&FATFS_logfile);			//Flush buffers
				f_truncate(&FATFS_logfile);		//Truncate the lenght - fix pre allocation
				f_close(&FATFS_logfile);		//Close any opened file
			}
			BootLoader();
		}
	}
}

