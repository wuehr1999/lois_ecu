/*
 * Uart sensor terminal emulation.
 */
#ifndef INC_TERMINALMODE_H_
#define INC_TERMINALMODE_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	TERMINAL_DISABLE = 0,
	TERMINAL_UART1 = 1,
	TERMINAL_UART2 = 2,
	TERMINAL_VCOM = 3,
	TERMINAL_NOCS = 4
}TERMINALMODE_Channels;

typedef struct TERMINALMODE_t
{
	TERMINALMODE_Channels input;
}TERMINALMODE_t;

void TERMINALMODE_Config(TERMINALMODE_t* t, TERMINALMODE_Channels input);
bool TERMINALMODE_IsInput(TERMINALMODE_t* t, TERMINALMODE_Channels channel);
bool TERMINALMODE_IsEnabled(TERMINALMODE_t* t);

#endif /* INC_TERMINALMODE_H_ */
