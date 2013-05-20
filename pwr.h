#pragma once
#include <string.h>
#include "stm32f10x.h"
#include "main.h"
#include "Util/fat_fs/inc/diskio.h"
#include "Util/fat_fs/inc/ff.h"

void setuppwr(void);
void shutdown(void);
void disable_pin(void);
void shutdown_filesystem(uint8_t reason, uint8_t file_flags);
