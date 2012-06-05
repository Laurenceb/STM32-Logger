#include "stm32f10x.h"
#pragma once

#define bytes_in_buff(buffer) !((buffer)->head==(buffer)->tail)

typedef struct{
	uint16_t head;
	uint16_t tail;
	uint16_t size;
	uint32_t* data;
} buff_type;

//Functions
void Add_To_Buffer(uint32_t data,buff_type* buffer);
uint8_t Get_From_Buffer(uint32_t* data,buff_type* buffer);
void Empty_Buffer(buff_type* buffer);
void init_buffer(buff_type* buff, uint16_t size);
