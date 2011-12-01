#include "pressure.h"
#include "../adc.h"

#define DIFF_GAIN 1.0/(float)113.23			/*pressure sensor 23mv/10v at 5psi, +60.1x inst amp gain, +12bit adc*/

static float pressure_offset;			//zero offset - calibrated at power on

void calibrate_sensor(void) {
	float pressoff=0;
	for(uint8_t n=0;n<250;n++)
		pressoff+=conv_adc_diff()/250.0;
	pressure_offset=pressoff;
}


float conv_adc_diff(void) {
	uint16_t p=readADC1(1);
	return 	(DIFF_GAIN*(float)p)-pressure_offset;
}
