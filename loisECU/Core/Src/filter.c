#include <filter.h>

void MOVINGAVG_Init(MovingAvg_t* filter, int* buffer, uint32_t size)
{
	filter->buffer = buffer;
	filter->size = size;
	filter->position = 0;
	filter->sum = 0;
	filter->output = 0;

	for(uint32_t i = 0; i < filter->size; i++)
	{
		filter->buffer[i] = 0;
	}
}

void MOVINGAVG_AddSample(MovingAvg_t* filter, int sample)
{
	filter->buffer[filter->position] = sample;
	filter->position ++;
	if(filter->position >= filter->size)
	{
		filter->position = 0;
	}
}

int MOVINGAVG_Calculate(MovingAvg_t* filter)
{
	filter->sum = 0;
	for(uint32_t i = 0; i < filter->size; i++)
	{
		filter->sum += filter->buffer[i];
	}

	filter->output = filter->sum / filter->size;

	return filter->output;
}
