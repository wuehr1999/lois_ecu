/*
 * Interface for communicating with host.
 */
#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "queue.h"
#include "robotparameters.h"
#include "timer.h"

#define INTERFACE_MSG_LEN 12

typedef struct INTERFACE_Data_t
{
	int instruction;
	uint16_t val1, val2;
}INTERFACE_Data_t;

typedef struct INTERFACE_Periodic_t
{
	uint8_t id;
	uint16_t period;
}INTERFACE_Periodic_t;

void INTERFACE_Parse(QUEUE_t* q);
int INTERFACE_CreateMessage16(char* messagePointer, int instruction, uint16_t val1, uint16_t val2);
int INTERFACE_CreateMessage8(char* messagePointer, int instruction, uint8_t val1, uint8_t val2, uint8_t val3, uint8_t val4);
int INTERFACE_CreateMessage32(char* messagePointer, int instruction, uint32_t val);
int INTERFACE_CreateMessageFloat(char* messagePointer, int instruction, float val);
void INTERFACE_Process(INTERFACE_Data_t* dat);
void INTERFACE_SendMessage(char* message, int len);

uint32_t uint32_2x16(uint16_t val1, uint16_t val2);
float float_2x16(uint16_t val1, uint16_t val2);

void INTERFACE_Init();
bool INTERFACE_AddPeriodic(uint8_t id, uint16_t period);
bool INTERFACE_SetPeriod(uint8_t id, uint16_t period);
void INTERFACE_ProcessPeriodics();
void INTERFACE_PeriodicRequired(INTERFACE_Periodic_t* p);
void INTERFACE_ResetPeriodics();
void INTERFACE_EnableCRC(bool enable);

#endif /* INC_INTERFACE_H_ */
