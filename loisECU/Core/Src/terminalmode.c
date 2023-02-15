#include "terminalmode.h"

void TERMINALMODE_Config(TERMINALMODE_t* t, TERMINALMODE_Channels input)
{
	t->input = input;
}

bool TERMINALMODE_IsInput(TERMINALMODE_t* t, TERMINALMODE_Channels channel)
{
	return channel == t->input;
}

bool TERMINALMODE_IsEnabled(TERMINALMODE_t* t)
{
	return (TERMINAL_DISABLE != t->input) && (TERMINAL_NOCS != t->input);
}
