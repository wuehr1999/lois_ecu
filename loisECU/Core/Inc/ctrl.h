/*
 * PID Controller core
 */
#ifndef INC_CTRL_H_
#define INC_CTRL_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * @brief Controller parameters
 */
typedef struct CTRL_t
{
  volatile float Ka, Kp, Tn, Td, T; // Continuous and sampling time
  volatile float d0, d1, d2, c1, c2; // Discrete coefficients
  volatile float ek_1, ek_2, uk_1, uk_2; // Discrete memory banks
  volatile float ek, uk; // Discrete input and output
  volatile float maxIn, maxOut; // Range limiters
  volatile float e, e_sum;
}CTRL_t;

/*
 * @brief Initializes controller
 *
 * @param Ka
 * @param Kp
 * @param Tn
 * @param Td
 * @param T 		Sampling time in seconds
 * @param maxIn 	Input value limit
 * @param maxOut 	Output value limit
 * @param ctrl 		Controller
 */
inline void CTRL_Init(float Ka, float Kp, float Tn, float Td, float T, float maxIn, float maxOut, CTRL_t* ctrl);

/*
 * @brief Controller update calculation
 * @brief Needs to be calld according to sampling time.
 *
 * @param curr 		Feedback
 * @param dest 		Destination
 * @param ctrl 		Controller
 * @param restart 	Do reset of coefficients
 */
inline float CTRL_Calculate(float curr, float dest, CTRL_t* ctrl, bool restart);

#endif /* INC_CTRL_H_ */
