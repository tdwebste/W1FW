#ifndef SENSOR_H__
#define SENSOR_H__

#include <stdint.h>
#include "main.h"

// debug log enble control
//#define SENSOR_DEBUG_EN
#undef SENSOR_DEBUG_EN

#ifdef SENSOR_DEBUG_EN
		#define W1_SENSOR_DEBUG(x)		W1_DEBUG(x)
#else
		#define W1_SENSOR_DEBUG(x)		do{} while(0);
#endif

/**@brief Function for making the ADC start a battery level conversion.
 */
void sensor_start(void);
void battery_start(void);
void ADC_Disable(void);
void check_step(uint16_t data);

#endif // SENSOR_H__

