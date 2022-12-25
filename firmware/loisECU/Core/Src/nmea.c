#include "nmea.h"

float NMEA_LocToDegs(char* data)
{
	char convStr[20];
	char* dot;
	int dotIndex;
	dot  = strchr(data, '.');
	dotIndex = (int)(dot - data);
	strncpy(convStr, data, dotIndex - 2);
	convStr[dotIndex - 2] = '\0';
	float degrees = atof(convStr);
	strcpy(convStr, &data[dotIndex - 2]);
	float minutes = atof(convStr);
	return degrees + minutes / 60.0;
	free(dot);
}

void NMEA_DecodeRMC(NMEA_InputData_t* n)
{
	NMEA_RMCData_t rmcData;

	char convStr[20];
	rmcData.hours = NMEA_INVALID;
	rmcData.minutes = NMEA_INVALID;
	rmcData.seconds = NMEA_INVALID;
	rmcData.latitude = NMEA_INVALID;
	rmcData.longitude = NMEA_INVALID;
	rmcData.speed_kmh = NMEA_INVALID;
	rmcData.course_deg = NMEA_INVALID;
	rmcData.day = NMEA_INVALID;
	rmcData.month = NMEA_INVALID;
	rmcData.year = NMEA_INVALID;
	for(int i = 0; i < 13; i++)
	{
		if('\0' != n->tokens[i][0])
		{
			switch(i)
			{
				case 1:	// TIME
					convStr[2] = '\0';
					convStr[0] = n->tokens[i][0];
					convStr[1] = n->tokens[i][1];
					rmcData.hours = atoi(convStr);
					convStr[0] = n->tokens[i][2];
					convStr[1] = n->tokens[i][3];
					rmcData.minutes = atoi(convStr);
					convStr[0] = n->tokens[i][4];
					convStr[1] = n->tokens[i][5];
					rmcData.seconds = atoi(convStr);
					break;
				case 2: // STATUS
					if('A' == n->tokens[i][0])
					{
						rmcData.isValid = true;
					}
					else if('V' == n->tokens[i][0])
					{
						rmcData.isValid = false;
					}
					break;
				case 3: // LAT
					rmcData.latitude = NMEA_LocToDegs(&n->tokens[i][0]);
					break;
				case 4: // NS
					if('N' == n->tokens[i][0])
					{
						rmcData.isNorth = true;
					}
					else if('S' == n->tokens[i][0])
					{
						rmcData.isNorth = false;
						rmcData.latitude *= -1;
					}
					break;
				case 5: // LON
					rmcData.longitude = NMEA_LocToDegs(&n->tokens[i][0]);
					break;
				case 6: // EW
					if('E' == n->tokens[i][0])
					{
						rmcData.isEast = true;
					}
					else if('w' == n->tokens[i][0])
					{
						rmcData.isEast = false;
						rmcData.longitude *= -1;
					}
					break;
				case 7: // SPEED
					strcpy(convStr, &n->tokens[i][0]);
					rmcData.speed_kmh = 1.852 * atof(convStr);
					break;
				case 8: // COURSE
					strcpy(convStr, &n->tokens[i][0]);
					rmcData.course_deg = atof(convStr);
					break;
				case 9: // DATE
					convStr[2] = '\0';
					convStr[0] = n->tokens[i][0];
					convStr[1] = n->tokens[i][1];
					rmcData.day = atoi(convStr);
					convStr[0] = n->tokens[i][2];
					convStr[1] = n->tokens[i][3];
					rmcData.month = atoi(convStr);
					convStr[0] = n->tokens[i][4];
					convStr[1] = n->tokens[i][5];
					rmcData.year = atoi(convStr);
					break;
				default: break;
			}
		}
	}

	NMEA_CallbackRMC(&rmcData);
}

void NMEA_DecodeGGA(NMEA_InputData_t* n)
{
	NMEA_GGAData_t ggaData;

	char convStr[20];

	ggaData.hours = NMEA_INVALID;
	ggaData.minutes = NMEA_INVALID;
	ggaData.seconds = NMEA_INVALID;
	ggaData.latitude = NMEA_INVALID;
	ggaData.longitude = NMEA_INVALID;
	ggaData.nrSatellites = NMEA_INVALID;
	ggaData.altitude = NMEA_INVALID;
	ggaData.hdop = NMEA_INVALID;

	for(int i = 0; i < 13; i++)
	{
		if('\0' != n->tokens[i][0])
		{
			switch(i)
			{
				case 1:	// TIME
					convStr[2] = '\0';
					convStr[0] = n->tokens[i][0];
					convStr[1] = n->tokens[i][1];
					ggaData.hours = atoi(convStr);
					convStr[0] = n->tokens[i][2];
					convStr[1] = n->tokens[i][3];
					ggaData.minutes = atoi(convStr);
					convStr[0] = n->tokens[i][4];
					convStr[1] = n->tokens[i][5];
					ggaData.seconds = atoi(convStr);
					break;
				case 2: // LAT
					ggaData.latitude = NMEA_LocToDegs(&n->tokens[i][0]);
					break;
				case 3: // NS
					if('N' == n->tokens[i][0])
					{
						ggaData.isNorth = true;
					}
					else if('S' == n->tokens[i][0])
					{
						ggaData.isNorth = false;
						ggaData.latitude *= -1;
					}
					break;
				case 4: // LON
					ggaData.longitude = NMEA_LocToDegs(&n->tokens[i][0]);
					break;
				case 5: // EW
					if('E' == n->tokens[i][0])
					{
						ggaData.isEast = true;
					}
					else if('w' == n->tokens[i][0])
					{
						ggaData.isEast = false;
						ggaData.longitude *= -1;
					}
					break;
				case 6: // QUALITY
					ggaData.quality = atoi(&n->tokens[i][0]);
					break;
				case 7: // SATELLITES
					ggaData.nrSatellites = atoi(&n->tokens[i][0]);
					break;
				case 8: // HDOP
					ggaData.hdop = atoi(&n->tokens[i][0]);
					break;
				case 9: // ALTITUDE
					ggaData.altitude = atoi(&n->tokens[i][0]);
					break;
				case 11: // GEOIDAL_SEP
					ggaData.geoidalSeparation = atoi(&n->tokens[i][0]);
					break;
				default: break;
			}
		}
	}
	NMEA_CallbackGGA(&ggaData);
}

void NMEA_DecodeHDT(NMEA_InputData_t* n)
{
	NMEA_HDTData_t hdtData;
	for(int i = 0; i < 3; i++)
	{
		switch(i)
		{
			case 1: // HEADING
					hdtData.heading = atoff(&n->tokens[i][0]);
				break;
			case 2: // T
					hdtData.T = ('T' == n->tokens[i][0]);
				break;

			default: break;
		}
	}
	NMEA_CallbackHDT(&hdtData);
}

bool NMEA_IsChecksumValid(char* string)
{
	int csPos = 0;
	int cs = 0;
	for(int i = 1; i < strlen(string); i++)
	{
		if('*' == string[i])
		{
			csPos = i + 1;
			break;
		}
		else
		{
			cs = cs ^ (int)string[i];
		}
	}

	return ((int)strtol(&string[csPos], NULL, 16) == cs);
}

void NMEA_Init(NMEA_InputData_t* n)
{
	n->pos = 0;
}

void NMEA_Parse(char c, NMEA_InputData_t* n)
{
	if('$' == c || NMEA_SENTENCE_MAXLEN <= n->pos)
	{
		n->pos = 0;
	}
	n->input[n->pos] = c;
	n->pos++;
	if('\n' == c)
	{
		if(NMEA_IsChecksumValid(n->input))
		{
			n->tokenSize = 0;
			int tokenPos = 0;

			for(int i = 0; i < n->pos; i++)
			{
				if(',' == n->input[i] || '\n' == n->input[i])
				{
					n->tokens[n->tokenSize][tokenPos] = '\0';
					n->tokenSize++;
					tokenPos = 0;
				}
				else
				{
					n->tokens[n->tokenSize][tokenPos] = n->input[i];
					tokenPos++;
				}
			}

			if(0 == strcmp("$GPRMC", &n->tokens[0][0]) || 0 == strcmp("$GNRMC", &n->tokens[0][0]))
			{
				NMEA_DecodeRMC(n);
			}
			else if(0 == strcmp("$GPGGA", &n->tokens[0][0]) || 0 == strcmp("$GNGGA", &n->tokens[0][0]))
			{
				NMEA_DecodeGGA(n);
			}
			else if(0 == strcmp("$HCHDT", &n->tokens[0][0]))
			{
				NMEA_DecodeHDT(n);
			}
		}
	}
}

__attribute__ ((weak))void NMEA_CallbackRMC(NMEA_RMCData_t* dat){}
__attribute__ ((weak))void NMEA_CallbackGGA(NMEA_GGAData_t* dat){}
__attribute__ ((weak))void NMEA_CallbackHDT(NMEA_HDTData_t* dat){}
