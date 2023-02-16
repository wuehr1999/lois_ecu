/*
 * adcfilter.c
 *
 *  Created on: Aug 15, 2021
 *      Author: jonas
 */
#include "adcfilter.h"

void ADCFILTER_Process(uint32_t* samples, uint32_t size)
{
	for(int i = 0; i < ADC_CHANNELS; i++)
	{
		ADCFILTER_Values[i] = 0;
	}

	for(int k = 0; k < size; k += ADC_CHANNELS)
	{
		for(int i = 0; i < ADC_CHANNELS; i++)
		{
			ADCFILTER_Values[i] += samples[k + i];
		}
	}

	for(int i = 0; i < ADC_CHANNELS; i++)
	{
		ADCFILTER_Values[i] = ADCFILTER_Values[i] / (size / ADC_CHANNELS);
	}
}
