#include "wire.h"

void WIRE_Init(WIRE_t* w,
		GPIO_TypeDef* sdaGPIO, uint16_t sdaPin,
		GPIO_TypeDef* sclGPIO, uint16_t sclPin,
		uint32_t delay,
		QUEUE_t* receiveBuf, QUEUE_t* sendBuf)
{
	w->sdaGPIO = sdaGPIO;
	w->sdaPIN = sdaPin;
	w->sclGPIO = sclGPIO;
	w->sclPIN = sclPin;
	w->delay = delay;
	w->in = receiveBuf;
	w->out = sendBuf;
}

void setBit(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
}

void resetBit(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
}

uint8_t gpioReadBit(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	return (uint16_t)HAL_GPIO_ReadPin(GPIOx, pin);
}

void wait(uint32_t delay)
{
	for(uint32_t i = 0; i < delay; i++)
	{
		asm("nop");
	}
}

void sdaInit(WIRE_t* w, bool output)
{
	GPIO_InitTypeDef init;
	init.Speed = GPIO_SPEED_FREQ_HIGH;
	init.Pin = w->sdaPIN;

	if(output)
	{
		init.Mode = GPIO_MODE_OUTPUT_OD;
	}
	else
	{
		init.Mode = GPIO_MODE_INPUT;
	}

	HAL_GPIO_Init(w->sdaGPIO, &init);
}

void sclInit(WIRE_t* w)
{
	GPIO_InitTypeDef init;
	init.Speed = GPIO_SPEED_FREQ_HIGH;
	init.Mode = GPIO_MODE_OUTPUT_OD;
	init.Pin = w->sclPIN;
	HAL_GPIO_Init(w->sclGPIO, &init);
}

void sdaHigh(WIRE_t* w)
{
	setBit(w->sdaGPIO, w->sdaPIN);
}

void sdaLow(WIRE_t* w)
{
	resetBit(w->sdaGPIO, w->sdaPIN);
}

void sdaOut(WIRE_t* w, uint8_t dat)
{
	if(dat)
	{
		sdaHigh(w);
	}
	else
	{
		sdaLow(w);
	}
}

void sclHigh(WIRE_t* w)
{
	setBit(w->sclGPIO, w->sclPIN);
}

void sclLow(WIRE_t* w)
{
	resetBit(w->sclGPIO, w->sclPIN);
}

void sclCycle(WIRE_t* w)
{
	sclHigh(w);
	wait(w->delay);
	sclLow(w);
}

void startCondition(WIRE_t* w)
{
	sdaHigh(w);
	sclHigh(w);
	wait(w->delay << 1);
//	wait(w->delay);
	sdaLow(w);
	wait(w->delay);
	sclLow(w);
}

void stopCondition(WIRE_t* w)
{
	sdaLow(w);
	sclHigh(w);
	wait(w->delay<<1);
//	wait(w->delay);
	sdaHigh(w);
	wait(w->delay);
}

uint8_t sdaRead(WIRE_t* w)
{
	return gpioReadBit(w->sdaGPIO, w->sdaPIN);
}

bool isAck(WIRE_t*w)
{
	unsigned int tmp;
	bool ack = false;

	sdaInit(w, false);
	wait(w->delay << 2);
//	wait(w->delay);
	sclHigh(w);
	wait(w->delay << 2);
//	wait(w->delay);

	for(int i = 0; i < 10; i++)
	{
		tmp = !(sdaRead(w));
		if(tmp)
		{
			ack = true;
			break;
		}
	}
	sclLow(w);
	sdaInit(w, true);
	wait(w->delay<<2);
//	wait(w->delay);
	return ack;
}

void WIRE_Begin(WIRE_t* w)
{
	sclInit(w);
	sdaInit(w, true);
	sdaHigh(w);
	sclHigh(w);
}

void WIRE_BeginTransmission(WIRE_t*w, uint8_t address)
{
	uint8_t addr = (address << 1);
	addr |= 0x00;
	while(!QUEUE_IsEmpty(w->out))
	{
		QUEUE_Pop(w->out);
	}
	w->bufferFull = false;
	QUEUE_Push(addr, w->out);
}

uint8_t WIRE_WriteByte(WIRE_t* w, uint8_t dat)
{
	if(!QUEUE_IsFull(w->out))
	{
		QUEUE_Push(dat, w->out);
	}
	else
	{
		w->bufferFull = true;
	}
	return QUEUE_Size(w->out);
}

uint8_t WIRE_WriteBytes(WIRE_t* w, uint8_t* dat, uint8_t len)
{
	int size = 0;
	for(int i = 0; i < len; i++)
	{
		size = WIRE_WriteByte(w, dat[i]);
	}
	return size;
}

uint8_t WIRE_EndTransmission(WIRE_t* w, bool stop)
{
	uint8_t error = WIRE_SUCCESS;
	if(w->bufferFull)
	{
		error = WIRE_BUFFEROVERFLOW;
	}
	else
	{
		sdaInit(w, true);
		startCondition(w);
		wait(w->delay);
		int ctr = 0;
		while(!QUEUE_IsEmpty(w->out))
		{
			uint8_t dat = QUEUE_Pop(w->out);
			for(int i = 7; i >= 0; i--)
			{
				sdaOut(w, dat & (1 << i));
				wait(w->delay);
				sclCycle(w);
				wait(w->delay);
			}
			bool ack = isAck(w);
			if(!ack)
			{
				if(0 == ctr)
				{
					error = WIRE_NACK_ADDR;
				}
				else
				{
					error = WIRE_NACK_DATA;
				}
				ctr++;
			}
		}

		wait(w->delay);

		if(stop)
		{
			w->restarted = false;
			stopCondition(w);
		}
		else
		{
			w->restarted = true;
			sdaHigh(w);
			wait(w->delay << 1);
			sclHigh(w);
			wait(w->delay << 1);
			sdaLow(w);
			wait(w->delay);
			sclLow(w);
		}
	}

	return error;
}

uint8_t WIRE_RequestFrom(WIRE_t* w, uint8_t address, uint8_t len, bool stop)
{
	uint8_t addr = address;
	addr = addr << 1;
	addr = addr | 0x01;

	sdaInit(w, true);
	if(!w->restarted)
	{
		startCondition(w);
	}
	wait(w->delay);
	for(int i = 7; i >= 0; i--)
	{
		sdaOut(w, addr & (1 << i));
		wait(w->delay);
		sclCycle(w);
		wait(w->delay);
	}
	bool ack = isAck(w);
	if(!ack)
	{
		return 0;
	}

	sclLow(w);
	sdaInit(w, true);
	wait(w->delay<<2);
//	wait(w->delay);

	while(!QUEUE_IsEmpty(w->in))
	{
		QUEUE_Pop(w->in);
	}

	w->bufferFull = false;
	sdaInit(w, false);
	uint8_t byte;
	for(int i = 0; i < len; i++)
	{
			byte = 0;
			for(int x = 0; x < 8; x++)
			{
					sclHigh(w);

					byte = byte << 1;
					if(sdaRead(w))
					{
						byte |= 0x01;
					}
					wait(w->delay);
					sclLow(w);
					wait(w->delay);
			}
			if(!QUEUE_IsFull(w->in))
			{
				QUEUE_Push(byte, w->in);
			}
			else
			{
				w->bufferFull = true;
			}
			sdaInit(w, true);
			if(i == len -1)
			{
				sdaHigh(w);
			}
			else
			{
				sdaLow(w);
			}
			wait(w->delay);
			sclCycle(w);
			wait(w->delay);
			sdaLow(w);
			wait(w->delay);
			sdaInit(w, false);
	}
	sdaInit(w, true);
	if(stop)
	{
		stopCondition(w);
	}

	return QUEUE_Size(w->in);
}

bool WIRE_IsAvailable(WIRE_t* w)
{
	return !QUEUE_IsEmpty(w->in);
}

uint8_t WIRE_Read(WIRE_t* w)
{
	if(!QUEUE_IsEmpty(w->in))
	{
		return QUEUE_Pop(w->in);
	}
	else
	{
		return 0;
	}
}
