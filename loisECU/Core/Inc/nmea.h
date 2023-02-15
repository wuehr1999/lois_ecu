/*
 * NMEA Parser
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define NMEA_INVALID -1000
#define NMEA_SENTENCE_MAXLEN 80
#define NMEA_SENTENCE_MAXTOKENS 20

/**
 * @brief Input data container with tokens
 */
typedef struct NMEA_InputData_t
{
	char input[NMEA_SENTENCE_MAXLEN];
	char tokens[NMEA_SENTENCE_MAXTOKENS][NMEA_SENTENCE_MAXLEN];
	int tokenSize;
	int pos;
}NMEA_InputData_t;

/*
 * @brief RMC data container
 */
typedef struct NMEA_RMCData_t
{
	int hours, minutes, seconds;
	bool isValid;
	float latitude, longitude;
	bool isNorth, isEast;
	float speed_kmh;
	float course_deg;
	int day, month, year;
}NMEA_RMCData_t;

/*
 * @brief GGA data container
 */
typedef struct NMEA_GGAData_t
{
	int hours, minutes, seconds;
	float latitude, longitude;
	bool isNorth, isEast;
	int quality, nrSatellites;
	float hdop;
	float altitude;
	float geoidalSeparation;
}NMEA_GGAData_t;


/*
 * @brief HDT data container
 */
typedef struct NMEA_HDTData_t
{
	float heading;
	bool T;
}NMEA_HDTData_t;

/*
 * @brief Parser init
 *
 * @param init Input data
 */
void NMEA_Init(NMEA_InputData_t* n);

/*
 * @brief Parser execution. Ends in callback if data is valid.
 *
 * @param c Next character from device
 * @param n Input data handle
 */
void NMEA_Parse(char c, NMEA_InputData_t* n);

/* Callbacks if data is ready */
void NMEA_CallbackRMC(NMEA_RMCData_t* dat);
void NMEA_CallbackGGA(NMEA_GGAData_t* dat);
void NMEA_CallbackHDT(NMEA_HDTData_t* dat);

#endif /* INC_NMEA_H_ */
