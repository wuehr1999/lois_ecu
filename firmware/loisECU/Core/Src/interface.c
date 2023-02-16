#include "interface.h"

char messageBuffer[ROBOT_MESSAGE_MAXLEN];
int messagePosition = 0;
char inStr[ROBOT_MESSAGE_INSTRUCTIONBYTES * 2 + 1];
char datStr[ROOBOT_MESSAGE_DATABYTES + 1];
char csStr[ROBOT_MESSAGE_CSBYTES + 1];

INTERFACE_Periodic_t periodics[MEMDEPTH_PERIODICS];
TIMER_t timers[MEMDEPTH_PERIODICS];

bool enableCRC;

int calculateCRC(char* message)
{
	int cs = 0;
	for(int i = 0; i < 1 + ROBOT_MESSAGE_INSTRUCTIONBYTES * 2 + ROOBOT_MESSAGE_DATABYTES * 2; i ++)
	{
		cs = cs ^ (int)message[i];
	}
	return cs;
}

void INTERFACE_Parse(QUEUE_t* q)
{
	static  char current = 0;

	while(!QUEUE_IsEmpty(q))
	{
		current = QUEUE_Pop(q);
		if(':' == current || INTERFACE_MSG_LEN + (enableCRC ? 2 : 0) <= messagePosition)
		{
			messagePosition = 0;
		}
		messageBuffer[messagePosition] = current;
		messagePosition++;
		if('\n' == current && INTERFACE_MSG_LEN + (enableCRC ? 2 : 0) == messagePosition)
		{

			csStr[ROBOT_MESSAGE_CSBYTES * 2] = '\0';
			int cs = 0;
			if(enableCRC)
			{
				memcpy(csStr, &messageBuffer[1 + ROBOT_MESSAGE_INSTRUCTIONBYTES * 2 + ROOBOT_MESSAGE_DATABYTES * 2], ROBOT_MESSAGE_INSTRUCTIONBYTES * 2 * sizeof(char));
				cs = strtol(csStr, NULL, 16);
			}
			if(!enableCRC || cs == calculateCRC(messageBuffer))
			{
				INTERFACE_Data_t dat;
				inStr[ROBOT_MESSAGE_INSTRUCTIONBYTES * 2] = '\0';
				datStr[ROOBOT_MESSAGE_DATABYTES * 2] = '\0';
				memcpy(inStr, &messageBuffer[1], ROBOT_MESSAGE_INSTRUCTIONBYTES * 2 * sizeof(char));
				dat.instruction = (int)strtol(inStr, NULL, 16);
				memcpy(datStr, &messageBuffer[1 + ROBOT_MESSAGE_INSTRUCTIONBYTES * 2], ROOBOT_MESSAGE_DATABYTES * sizeof(char));
				dat.val1 = strtol(datStr, NULL, 16);
				memcpy(datStr, &messageBuffer[1 + ROBOT_MESSAGE_INSTRUCTIONBYTES * 2 + ROOBOT_MESSAGE_DATABYTES], ROOBOT_MESSAGE_DATABYTES * sizeof(char));
				dat.val2 = strtol(datStr, NULL, 16);
				INTERFACE_Process(&dat);
			}
		}
	}
}

__attribute__ ((weak)) void INTERFACE_Process(INTERFACE_Data_t* dat){}

int INTERFACE_CreateMessage16(char* messagePointer, int instruction, uint16_t val1, uint16_t val2)
{
	char buf[ROBOT_MESSAGE_MAXLEN];
	int len = sprintf(buf, ":%02x%04x%04x", instruction, val1, val2);
	int cs = calculateCRC(buf);
	len = sprintf(messagePointer, "%s%02x\n", buf, cs);
	return len;
}

int INTERFACE_CreateMessage8(char* messagePointer, int instruction, uint8_t val1, uint8_t val2, uint8_t val3, uint8_t val4)
{
	return INTERFACE_CreateMessage16(messagePointer, instruction,
			(uint16_t)((val1 << 8) | val2), (uint16_t)((val3 << 8) | val4));
}

int INTERFACE_CreateMessage32(char* messagePointer, int instruction, uint32_t val)
{

	return INTERFACE_CreateMessage16(messagePointer, instruction, (uint16_t)(val >> 16), (uint16_t)(val & 0xFFFF));
}

int INTERFACE_CreateMessageFloat(char* messagePointer, int instruction, float val)
{
	uint32_t v;
	memcpy(&v, &val, sizeof(float));
	return INTERFACE_CreateMessage32(messagePointer, instruction, v);
}

__attribute__ ((weak))void INTERFACE_SendMessage(char* message, int len){}

uint32_t uint32_2x16(uint16_t val1, uint16_t val2)
{
	uint32_t val = (uint32_t)val1;
	val = val << 16;
	val += (uint32_t)val2;

	return val;
}

float float_2x16(uint16_t val1, uint16_t val2)
{
	uint32_t val = uint32_2x16(val1, val2);

    float f;
    memcpy(&f, &val, sizeof(uint32_t));

    return f;
}

void INTERFACE_Init()
{
	enableCRC = true;
	for(uint32_t i = 0; i < MEMDEPTH_PERIODICS; i++)
	{
		periodics[i].id = 0;
		periodics[i].period = ROBOT_PERIOD_NEVER;
	}
}

bool INTERFACE_AddPeriodic(uint8_t id, uint16_t period)
{
	static uint32_t cnt = 0;
	if(cnt < MEMDEPTH_PERIODICS)
	{
		periodics[cnt].id = id;
		periodics[cnt].period = period;
		if(ROBOT_PERIOD_ONCE != period && ROBOT_PERIOD_NEVER != period)
		{
			TIMER_Start(&timers[cnt]);
		}
		cnt++;
		return true;
	}
	else
	{
		return false;
	}
}

bool INTERFACE_SetPeriod(uint8_t id, uint16_t period)
{
	for(uint32_t i = 0; i < MEMDEPTH_PERIODICS; i++)
	{
		if(id == periodics[i].id)
		{
			periodics[i].period = period;
			if(ROBOT_PERIOD_ONCE != period && ROBOT_PERIOD_NEVER != period)
			{
				TIMER_Start(&timers[i]);
			}
			return true;
		}
	}

	return false;
}

void INTERFACE_ProcessPeriodics()
{
	INTERFACE_Periodic_t* current;
	for(uint32_t i = 0; i < MEMDEPTH_PERIODICS; i++){
	current = &periodics[i];
	if(ROBOT_PERIOD_NEVER != current->period
			&& (ROBOT_PERIOD_ONCE == current->period || TIMER_IsExpired(&timers[i], current->period)))
	{
		INTERFACE_PeriodicRequired(current);
			if(ROBOT_PERIOD_ONCE == current->period)
		{
			current->period = ROBOT_PERIOD_NEVER;
		}
		else
		{
			TIMER_Start(&timers[i]);
		}
	}}
}

void INTERFACE_ResetPeriodics()
{
	for(uint32_t i = 0; i < MEMDEPTH_PERIODICS; i++)
	{
		TIMER_Start(&timers[i]);
	}
}

void INTERFACE_EnableCRC(bool enable)
{
	enableCRC = enable;
}

__attribute__ ((weak))void INTERFACE_PeriodicRequired(INTERFACE_Periodic_t* p){}
