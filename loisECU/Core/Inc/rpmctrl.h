/*
 * RPM controller core
 */

#ifndef INC_RPMCTRL_H_
#define INC_RPMCTRL_H_

#include "ctrl.h"
#include "queue.h"
#include "robotparameters.h"
/*
 * @brief Data container
 */
typedef struct RPMCTRL_t
{
	volatile long eternalTicks; 			// Odometry tick counter
	volatile uint32_t tmpTicks; 	    	// Ticks between two control loop interrupts
	volatile float corrfacShort, corrfacLong; 		// Edge correction factors
	volatile uint64_t sum; 					// Ticks time between two control loop interrupts
	volatile int dutyCycle; 				// Output motor dutycycle
	volatile int rpm, rpmDest, lastRpmDest; // Feedback and destination
	volatile uint32_t timestamp_us; 		// Timestamp
	volatile QUEUE_t* recordQueue; 			// Recording buffer
	uint32_t recordSamples; 				// Number of samples to record
	volatile bool isRecord;					// Enable recording
	volatile bool restart; 					// Restart controller
	volatile bool ignore; 					// Disable controller
	int steps; 								// Encoder steps per wheel revolution
	bool recordTorque; 						// Do torque recording, not rpm
	uint16_t avgTorque; 					// Average torque value for one controller iteration
}RPMCTRL_t;

/**
 * @brief Initialize RPM controller
 *
 * @param steps 		Encoder steps per wheel revolution
 * @param corrfacShort	Correction factor for short edge
 * @param corrfacLong	Correction factor for long edge
 * @param rpmCtrl		RPM controller
 * @param ctrl			Preinitialized PID-controller
 */
void RPMCTRL_Init(int steps, float corrfacShort, float corrfacLong, RPMCTRL_t* rpmCtrl);

/**
 * @brief Tick update function.
 * @brief Needs to be called at every trigger edge.
 *
 * @param ctrl 			RPM controller
 * @param timestamp_us	Timestamp in microseconds. Timeroverflow is handled.
 * @param isLong		Indicate if it is a short or long edge.
 */
inline void RPMCTRL_Tick(RPMCTRL_t* ctrl, volatile uint32_t timestamp_us, bool isLong);

/*
 * @brief Setter for average Torque value
 *
 * @param ctrl 		RPM controller
 * @param torque	Relative torque/ current absolute value
 */
void RPMCTRL_SetAvgTorque(RPMCTRL_t* ctrl, uint16_t torque);

/*
 * @brief Update RPM controller output.
 * @brief This is the actual execution of the control loop.
 * @brief Needs to be called with sampling time.
 *
 * @parm ctrl RPM controller
 */
inline void RPMCTRL_Update(RPMCTRL_t* ctrl, CTRL_t* ctrl_base);

/*
 * @brief Set destination value
 *
 * @param ctrl RPM controller
 * @param dest Destination value in RPM
 */
void RPMCTRL_SetDest(RPMCTRL_t* ctrl, int dest, CTRL_t* ctrl_base);

/*
 * @brief Disable controller
 *
 * @param bool disable/ enable
 */
void RPMCTRL_Ignore(RPMCTRL_t* ctrl, bool ignore, CTRL_t* ctrl_base);

/*
 * @brief Start realtime measurement recording
 *
 * @param ctrl 			RPM controller
 * @param sample 		Number of measurements
 * @param sampleQueue 	Data buffer
 * @param torque		Do torque or rpm measurement
 */
void RPMCTRL_StartRecord(RPMCTRL_t* ctrl, uint32_t samples, QUEUE_t* sampleQueue, bool torque);

/*
 * @brief Check if record is done.
 *
 * @param ctrl 	RPM controller
 * @return		True if record is done
 */
bool RPMCTRL_IsRecordDone(RPMCTRL_t* ctrl);

/*
 * @brief Cancel running rpm record
 *
 * @param ctrl	RPM controller
 */
void RPMCTRL_StopRecord(RPMCTRL_t* ctrl);

#endif /* INC_RPMCTRL_H_ */
