/*
 * Software I2C oriented on arduino implementation.
 */
#ifndef INC_WIRE_H_
#define INC_WIRE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_gpio.h>

#include "queue.h"

#define WIRE_SUCCESS 0
#define WIRE_BUFFEROVERFLOW 1
#define WIRE_NACK_ADDR 2
#define WIRE_NACK_DATA 3

typedef struct WIRE_t
{
	GPIO_TypeDef* sdaGPIO;
	GPIO_TypeDef* sclGPIO;
	uint16_t sdaPIN, sclPIN;
	uint32_t delay;
	QUEUE_t* in;
	QUEUE_t* out;
	bool bufferFull;
	bool restarted;
}WIRE_t;

void WIRE_Init(WIRE_t* w,
		GPIO_TypeDef* sdaGPIO, uint16_t sdaPin,
		GPIO_TypeDef* sclGPIO, uint16_t sclPin,
		uint32_t delay,
		QUEUE_t* receiveBuf, QUEUE_t* sendBuf);

void WIRE_Begin(WIRE_t* w);
void WIRE_BeginTransmission(WIRE_t*w, uint8_t address);
uint8_t WIRE_WriteByte(WIRE_t* w, uint8_t dat);
uint8_t WIRE_WriteBytes(WIRE_t* w, uint8_t* dat, uint8_t len);
uint8_t WIRE_EndTransmission(WIRE_t* w, bool stop);
uint8_t WIRE_RequestFrom(WIRE_t* w, uint8_t address, uint8_t len, bool stop);
bool WIRE_IsAvailable(WIRE_t* w);
uint8_t WIRE_Read(WIRE_t* w);

#endif /* INC_WIRE_H_ */
