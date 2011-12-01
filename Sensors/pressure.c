#include "pressure.h"
#include "../adc.h"

#define DIFF_GAIN 1.0/(float)113.23			/*pressure sensor 23mv/10v at 5psi, +60.1x inst amp gain, +12bit adc*/

static float pressure_offset;			//zero offset - calibrated at power on

void calibrate_sensor(void) {
	uint32_t pressoff=0;
	register l;
	for(uint8_t n=1;n;n++) {		//take 256 samples from the pressure sensor
		pressoff+=readADC1(1);
		for(l=10000;l;l--);		//~1ms between samples
	}
	pressure_offset=(float)pressoff/(float)256.0;
}


float conv_adc_diff(void) {
	uint16_t p=readADC1(1);
	return 	(DIFF_GAIN)*((float)p-pressure_offset);
}
