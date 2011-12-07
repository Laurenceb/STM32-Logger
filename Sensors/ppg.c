#include "ppg.h"
#include "../Util/buffers.h"


PPG_LO_Filter(uint16_t* buff) {
	int32_t I=0,Q=0;	//I and Q integration bins
	for(uint16_t n=0;n<sizeof(buff)/2;) {//buffer size/2 must be a multiple of 4
		I+=buff[n++];
		Q+=buff[n++];
		I-=buff[n++];
		Q-=buff[n++];
	}
	Add_To_Buffer((uint32_t)sqrt((int64_t)I^2+(int64_t)Q^2),&Baseband_Buff);//find the magnitude using 64bit ints, add to buff
}
