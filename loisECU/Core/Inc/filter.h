/*
 * Implementation of moving average filter for 32-bit integers.
 */
#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include <stdint.h>
/*
 * @brief Filter data container
 */
typedef struct MovingAvg_t
{
	int* buffer;		// Pointer to filter buffer
	uint32_t size;		// Buffer size
	uint32_t position;	// Position in buffer
	int sum, output;	// Values sum and output
}MovingAvg_t;

/*
 * @brief Initialize filter.
 *
 * @param filter Filter container
 * @param buffer Preallocated sampling buffer
 * @param size 	 Size of buffer
 */
void MOVINGAVG_Init(MovingAvg_t* filter, int* buffer, uint32_t size);

/*
 * @brief Add sample to filter.
 *
 * @param filter Filter container
 * @param sample Sample value
 */
void MOVINGAVG_AddSample(MovingAvg_t* filter, int sample);

/*
 * @brief Do averaging over samples
 *
 * @param filter Filter container
 *
 * @return Filter value
 */
int MOVINGAVG_Calculate(MovingAvg_t* filter);

#endif /* INC_FILTER_H_ */
