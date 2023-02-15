/*
 * Queue buffer based on preallocated memory area
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * @brief Queue data containger
 */
typedef struct QUEUE_t
{
	uint32_t max, rear, front, elements; // Parameters
	char* buf;							 // Data buffer
}QUEUE_t;

/*
 * @brief Init queue
 *
 * @param q		Queue
 * @param buf 	Data buffer
 * @param max	Maximum 8-bit elements
 */
void QUEUE_Init(QUEUE_t* q, char* buf, uint32_t max);

/*
 * @brief FLush queue
 *
 * @param q Queue
 */
void QUEUE_Reset(QUEUE_t* q);

/*
 * @brief Check if there is space for a number of bytes
 *
 * @param bytes Number of bytes
 * @param q		Queue
 *
 * @return is free
 */
inline bool QUEUE_IsFree(uint32_t bytes, QUEUE_t* q);

/*
 * @brief Check if queue is full for a number of bytes
 *
 * @param bytes Number of bytes
 * @param q		Queue
 *
 * @return no space left
 */
inline bool QUEUE_IsFullN(uint32_t bytes, QUEUE_t* q);
/* IsFullN for 1 byte */
inline bool QUEUE_IsFull(QUEUE_t* q);
/* IsFullN for 2 bytes */
inline bool QUEUE_IsFull16(QUEUE_t* q);
/* IsFullN for 4 bytes */
inline bool QUEUE_IsFull32(QUEUE_t* q);

/*
 * @brief Check if queue is empty
 *
 * @param q Queue
 * @return 	is empty
 */
inline bool QUEUE_IsEmpty(QUEUE_t* q);

/* Push 1 byte */
inline void QUEUE_Push(char val, QUEUE_t* q);
/* Push 2 bytes */
inline void QUEUE_Push16(uint16_t val, QUEUE_t* q);
/* Push 4 bytes */
inline void QUEUE_Push32(uint32_t val, QUEUE_t* q);

/* Pop 1 byte */
inline char QUEUE_Pop(QUEUE_t* q);
/* Pop 2 bytes */
inline uint16_t QUEUE_Pop16(QUEUE_t* q);
/* Pop 4 bytes */
inline uint32_t QUEUE_Pop32(QUEUE_t* q);

/*
 * @param q Queue
 * @return Size in bytes
 */
inline uint32_t QUEUE_Size(QUEUE_t* q);

/*
 * @brief Get data pointer.
 *
 * @param q Queue
 *
 * @return Pointer to first valid data byte
 */
inline char* QUEUE_GetPointer(QUEUE_t* q);

/*
 * @brief Mark a number of bytes as enqueued.
 * @brief Check if enought space is left must be done.
 *
 * @param q 	Queue
 * @param bytes Number of bytes
 */
inline void QUEUE_SetEnqueued(QUEUE_t* q, uint32_t bytes);

/*
 * @brief Push multiple bytes by memory copy.
 *
 * @param q 	 Queue
 * @param buffer Input buffer
 * @param bytes  Number of bytes to push
 *
 * @return success
 */
inline bool QUEUE_Memcpy(QUEUE_t* q, void* buffer, uint32_t bytes);

#endif /* INC_QUEUE_H_ */
