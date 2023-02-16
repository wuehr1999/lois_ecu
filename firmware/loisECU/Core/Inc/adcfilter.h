/*
 * Moving average for processing DMA samples of ADC
 */

#ifndef INC_ADCFILTER_H_
#define INC_ADCFILTER_H_

#include <stdint.h>

#include "robotparameters.h"

/*
 * @brief Maps samples to ADC channels
 */
typedef enum
{
	ADC_BEMF12 = 0,		// Back EMF motor 1
	ADC_BEMF34 = 1,		// Back EMF motor 2
	ADC_IS12 = 2,		// Current/ torque motor 1
	ADC_IS34 = 3,		// Current/ rotque motor 2
	ADC_BATSENSE = 4	// Battery voltage
}ADCFILTER_Channels;

// Buffer for filter values
uint32_t ADCFILTER_Values[5];

/*
 * @brief Processing function. Fills ADCFILTER_Values buffer.
 *
 * @param samples 	DMA memory area
 * @param size 		Numer of samples to process
 */
void ADCFILTER_Process(uint32_t* samples, uint32_t size);

#endif /* INC_ADCFILTER_H_ */
