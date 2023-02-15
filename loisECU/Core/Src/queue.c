#include "queue.h"

void QUEUE_Init(QUEUE_t* q, char* buf, uint32_t max)
{
	q->max = max;
	q->elements = 0;
	q->buf = buf;
	q->rear = q->max - 1;
	q->front = 0;
}

void QUEUE_Reset(QUEUE_t* q)
{
	q->elements = 0;
	q->rear = q->max - 1;
	q->front = 0;
}

bool QUEUE_IsFree(uint32_t bytes, QUEUE_t* q)
{
	return (q->max - q->elements) >= bytes;
}

bool QUEUE_IsFullN(uint32_t bytes, QUEUE_t* q)
{
	return !QUEUE_IsFree(bytes, q);
}

bool QUEUE_IsFull(QUEUE_t* q)
{
	return QUEUE_IsFullN(1, q);
}

bool QUEUE_IsFull16(QUEUE_t* q)
{
	return QUEUE_IsFullN(2, q);
}

bool QUEUE_IsFull32(QUEUE_t* q)
{
	return QUEUE_IsFullN(4, q);
}

bool QUEUE_IsEmpty(QUEUE_t* q)
{
	return (0 == q->elements);
}

void QUEUE_Push(char val, QUEUE_t* q)
{
	q->rear = (q->rear + 1) % q->max;
	q->elements++;
	q->buf[q->rear] = val;
}

void QUEUE_Push16(uint16_t val, QUEUE_t* q)
{
	QUEUE_Push((char)(val & 0xff), q);
	QUEUE_Push((char)(val >> 8 & 0xff), q);
}

void QUEUE_Push32(uint32_t val, QUEUE_t* q)
{
	QUEUE_Push((char)(val & 0xff), q);
	QUEUE_Push((char)(val >> 8 & 0xff), q);
	QUEUE_Push((char)(val >> 16 & 0xff), q);
	QUEUE_Push((char)(val >> 24 & 0xff), q);
}


char QUEUE_Pop(QUEUE_t* q)
{
	char c = q->buf[q->front];
	q->front = (q->front + 1) % q->max;
	q->elements--;
	return c;
}

uint16_t QUEUE_Pop16(QUEUE_t* q)
{
	char b1 = QUEUE_Pop(q);
	char b0 = QUEUE_Pop(q);
	return (uint16_t)b0 << 8 | (uint16_t)b1;
}

uint32_t QUEUE_Pop32(QUEUE_t* q)
{
	char b3 = QUEUE_Pop(q);
	char b2 = QUEUE_Pop(q);
	char b1 = QUEUE_Pop(q);
	char b0 = QUEUE_Pop(q);
	return (uint32_t)b0 << 24 | (uint32_t)b1 << 16 | (uint32_t)b2 << 8 | (uint32_t)b3;
}

uint32_t QUEUE_Size(QUEUE_t* q)
{
	return q->elements;
}

char* QUEUE_GetPointer(QUEUE_t* q)
{
	return &q->buf[q->front];
}

void QUEUE_SetEnqueued(QUEUE_t* q, uint32_t bytes)
{
	for(uint32_t i = 0; i < bytes; i++)
	{
		q->rear = (q->rear + 1) % q->max;
		q->elements++;
	}
}

bool QUEUE_Memcpy(QUEUE_t* q, void* buffer, uint32_t bytes)
{
	if(!QUEUE_IsFree(bytes, q))
	{
		return false;
	}
	else
	{
		if(q->rear > q->front)
		{
			memcpy((void*)&q->buf[q->front], buffer, bytes);
		}
		else
		{
			int len1 = q->max - q->front;
			int len2 = bytes - len1;
			memcpy((void*)&q->buf[q->front], buffer, len1);
			if(len2 > 0)
			{
				memcpy((void*)&q->buf[0], buffer, len2);
			}
		}
		QUEUE_SetEnqueued(q, bytes);
		return true;
	}
}
