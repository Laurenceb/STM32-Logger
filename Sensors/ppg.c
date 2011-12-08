#include "ppg.h"
#include "../Util/buffer.h"


PPG_LO_Filter(uint16_t* buff) {
	int32_t I=0,Q=0;	//I and Q integration bins
	for(uint16_t n=0;n<sizeof(buff)/2;) {//buffer size/2 must be a multiple of 4
		I+=buff[n++];
		Q+=buff[n++];
		I-=buff[n++];
		Q-=buff[n++];
	}
	Add_To_Buffer((uint32_t)I,&Baseband_Buff);//add to buffer
	Add_To_Buffer((uint32_t)Q,&Baseband_Buff);
}
