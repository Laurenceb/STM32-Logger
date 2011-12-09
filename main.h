#include "stm32f10x.h"
#pragma once
#include "Util/fat_fs/inc/ff.h"
#include "Util/buffer.h"

#define PRE_SIZE 1000000ul	/*Preallocate size*/

#define delay()						\
do {							\
  register unsigned int i;				\
  for (i = 0; i < 10000000; ++i)			\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)

//function prototypes
void __fat_print_char(char c);
void __str_print_char(char c);
//globals
extern buff_type Buff;
//fatfs globals
extern volatile uint8_t file_opened;
extern FIL FATFS_logfile;
