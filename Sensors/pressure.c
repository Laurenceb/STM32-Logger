#include "pressure.h"
#include "../adc.h"

#define DIFF_GAIN 1.0/(float)113.23		/*pressure sensor 23mv/10v at 5psi, +60.1x inst amp gain, +12bit adc*/

static float pressure_offset;			//zero offset - calibrated at power on

/**
  * @brief  This function calibrates the adc offset on the pressure sensor
  * @param  None
  * @retval None
  */
void calibrate_sensor(void) {
	uint32_t pressoff=0;
	volatile uint32_t l;
	for(uint8_t n=1;n;n++) {		//take 256 samples from the pressure sensor
		pressoff+=readADC2(1);
		for(l=10000;l;l--);		//~1ms between samples
	}
	pressure_offset=(float)pressoff/(float)255.0;
}

/**
  * @brief  This function returns the converted pressure - blocking
  * @param  None
  * @retval Pressure in PSI
  */
float conv_adc_diff(void) {
	uint16_t p=readADC2(1);
	return 	(DIFF_GAIN)*((float)p-pressure_offset);
}

/**
  * @brief  This function returns the converted pressure from an unsigned integer
  * @param  uint16_t from ADC
  * @retval Pressure in PSI
  */
float conv_diff(uint16_t diff) {
	return 	(DIFF_GAIN)*((float)diff-pressure_offset);
}
