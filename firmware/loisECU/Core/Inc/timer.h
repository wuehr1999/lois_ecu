/*
 * Timer in ms resolution based on systick.
 */
#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"

/*
 * @brief Timestamp
 */
typedef struct TIMER_t
{
	uint32_t start;	// System time in ms
}TIMER_t;

/*
 * @brief Start timer.
 *
 * @param t Timestamp
 */
inline void TIMER_Start(TIMER_t* t);

/*
 * @brief Stop timer.
 *
 * @param t Timestamp
 *
 * @return Expired time in ms
 */
inline uint32_t TIMER_Stop(TIMER_t* t);

/*
 * @brief Check if timer has expired period.
 *
 * @param t 		Timestamp
 * @param period 	Period to check in ms.
 *
 * @return is exired
 */
inline bool TIMER_IsExpired(TIMER_t* t, uint32_t period);

#endif /* INC_TIMER_H_ */
