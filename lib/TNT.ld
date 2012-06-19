/*
Default linker script for STM32F103_512K_64K
Copyright RAISONANCE S.A.S. 2007
*/

/* include the common STMxxx sub-script */
INCLUDE "STM32F10x_COMMON.ld"

/* include the memory spaces definitions sub-script */
INCLUDE "TNT_512K_64K_DEF.ld"

/* include the sections management sub-script for FLASH mode */
INCLUDE "sections_FLASH.ld"
