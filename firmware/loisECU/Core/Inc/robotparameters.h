/*
 * Defines for robot parameter sets. This parameters are used as initial values and can be partly reconfigured by host.
 */

#ifndef INC_ROBOTPARAMETERS_H_
#define INC_ROBOTPARAMETERS_H_

#include "interface.h"

// Memory allocations
#define MEMDEPTH_INTERFACE 50 * INTERFACE_MSG_LEN	// Host Interface input
#define MEMDEPTH_WIRE 20							// Software I2C
#define MEMDEPTH_RECORD 4 * 512						// Realtime recording
#define MEMDEPTH_KVH_MOVINGAVG 3					// Compass moving average (1 means turned off filter)
#define MEMDEPTH_PERIODICS 6						// Host interface periodical messages
#define MEMDEPTH_KVH 100							// Compass input
#define INTERVAL_KVH 10								// Bytes received per interrupt
#define MEMDEPTH_GPS 200							// GPS input
#define INTERVAL_GPS 20								// Bytes received per interrupt

// ADC DMA setup
#define ADC_CHANNELS 5											// Number of channels
#define ADC_CONFTIME_US 60										// Sampling Time (must be measured)
#define ADC_SAMPLES 100 * ADC_CHANNELS							// Number of samples
#define ADC_PERIOD_MS = ADC_CONFTIME_US * ADC_SAMPLES / 1000	// Sampling period

// Powertrain setup
#define ROBOT_DUTYCYCLE_MAX 100	// Maximum motor dutycycle value
#define ROBOT_RPM_MAX 50		// Maximum motor rpm
#define ROBOT_ENCODER_STEPS 60  // Encoder steps per wheel revolution
#define ROBOT_RPMCTRL_T 0.2 	// Sampling time of rpm control loop
#define ROBOT_WHEELRADIUS_M 0.13
#define ROBOT_WHEELWIDTH_M 0.47

// Fallback edge correction parameters
#define ROBOT_LEFT_CORRFACSHORT 1.0761719
#define ROBOT_LEFT_CORRFACLONG 0.9338983
#define ROBOT_RIGHT_CORRFACSHORT 1.0954246
#define ROBOT_RIGHT_CORRFACLONG 0.9198685

// Fallback controller parameters
#define ROBOT_LEFT_KP 0.4412007
#define ROBOT_LEFT_TD 0.6494819
//#define ROBOT_LEFT_KP 0.5
//#define ROBOT_LEFT_TD 2.5
#define ROBOT_LEFT_TN 0.286456
#define ROBOT_RIGHT_KP 0.7316168
#define ROBOT_RIGHT_TD 0.6494819
//#define ROBOT_RIGHT_KP 0.5
//#define ROBOT_RIGHT_TD 2.5
#define ROBOT_RIGHT_TN 0.1382718

// Interface setup (message format is :%02d%04d%04d\n. First byte is instruction header followed by 2x2byte payload data.)
#define ROBOT_MESSAGE_MAXLEN 20
#define ROBOT_MESSAGE_START ':'
#define ROBOT_MESSAGE_END '\n'
#define ROBOT_MESSAGE_INSTRUCTIONBYTES 1
#define ROOBOT_MESSAGE_DATABYTES 4
#define ROBOT_MESSAGE_CSBYTES 1

// Instruction headers and periodics
#define ROBOT_INSTRUCTION_DUTYCYCLE 0x01
#define ROBOT_INSTRUCTION_RPM 0x02
#define ROBOT_INSTRUCTION_LED 0x03
#define ROBOT_INSTRUCTION_RECORDRPM 0x04
#define ROBOT_INSTRUCTION_EMSTOP 0x05

#define ROBOT_INSTRUCTION_LEFT_KP 0x06
#define ROBOT_INSTRUCTION_LEFT_TN 0x07
#define ROBOT_INSTRUCTION_LEFT_TD 0x08
#define ROBOT_INSTRUCTION_RIGHT_KP 0x09
#define ROBOT_INSTRUCTION_RIGHT_TN 0x0a
#define ROBOT_INSTRUCTION_RIGHT_TD 0x0b

#define ROBOT_INSTRUCTION_RECORDHEADING 0x0c

#define ROBOT_PERIOD_ONCE 0xfffe
#define ROBOT_PERIOD_NEVER 0xffff

#define ROBOT_PERIODIC_LATLON 0x0d
#define ROBOT_PERIOD_LATLON 1200

#define ROBOT_PERIODIC_TIME 0x0e
#define ROBOT_PERIOD_TIME 1000

#define ROBOT_PERIODIC_DATE 0x0f
#define ROBOT_PERIOD_DATE 0xffff

#define ROBOT_PERIODIC_HEADING 0x10
#define ROBOT_PERIOD_HEADING 400

#define ROBOT_INSTRUCTION_TERMINALMODE 0x11

#define ROBOT_PERIODIC_ENCODERS 0x12
#define ROBOT_PERIOD_ENCODERS 400

#define ROBOT_INSTRUCTION_LEFT_CORRFAC_SHORT 0x13
#define ROBOT_INSTRUCTION_LEFT_CORRFAC_LONG 0x14
#define ROBOT_INSTRUCTION_RIGHT_CORRFAC_SHORT 0x15
#define ROBOT_INSTRUCTION_RIGHT_CORRFAC_LONG 0x16

#define ROBOT_INSTRUCTION_RECORDCURRENT 0x17

#define ROBOT_INSTRUCTION_RESET 0x18

#define ROBOT_INSTRUCTION_BALLSHIT 0x19

#define ROBOT_PERIODIC_ODOM 0x20
#define ROBOT_PERIOD_ODOM 400

#endif /* INC_ROBOTPARAMETERS_H_ */
