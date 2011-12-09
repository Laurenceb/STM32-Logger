#include <math.h>
#include "ppg.h"
#include "../Util/buffer.h"
#include "../main.h"

//This should really be done with macros, but 64(decimate)*21(decimate)*14(adc clks)*6(adc clkdiv)=112896
// == 336*336 - so one change in the pwm reload value will give an orthogonal frequency to the baseband decimator 
//This will be called at 13.393KHz
void PPG_LO_Filter(uint16_t* buff) {
	int32_t I=0,Q=0;	//I and Q integration bins
	static uint8_t bindex;	//Baseband decimation index
	static uint32_t Frequency_Bin[1][2];//Only one frequency in use atm - consisting of and I and Q component
	for(uint16_t n=0;n<sizeof(buff)/2;) {//buffer size/2 must be a multiple of 4
		I+=buff[n++];
		Q+=buff[n++];
		I-=buff[n++];
		Q-=buff[n++];
	}
	//Now run the "baseband" decimating filter(s)
	Frequency_Bin[0][0]+=I;Frequency_Bin[0][1]+=Q;//Add the I and Q directly into the zero frequency bin
	//Other Bins for +ive or -ive frequencies go here - use a lookup table for the sin/cos samples
	if(++bindex==21) {//Decimation factor of 21 - 637.8Hz data output
		Add_To_Buffer((uint32_t)sqrt(pow((int64_t)Frequency_Bin[0][0],2)+pow((int64_t)Frequency_Bin[0][1],2)),&Buff);
		//Other frequencies corresponding to different LEDs could go here - use different buffers maybe?
		memset(Frequency_Bin,0,sizeof(Frequency_Bin));//Zero everything
	}
}
