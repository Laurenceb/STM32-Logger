#include "temperature.h"


float convert_die_temp(uint16_t adcval) {
	return ((float)adcval*TEMPERATURE_GAIN)-TEMPERATURE_OFFSET;
}
