/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_OFFSET 1750
#define SERVO_RANGE 6500
#define BALLSHITTER_TIME 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Host communication interface
char interfaceBuf[MEMDEPTH_INTERFACE];
QUEUE_t interfaceQueue;
INTERFACE_Data_t interfaceData;

// RPM control and odometry
ODOMETRY_t odom;
RPMCTRL_t ctrlLeft, ctrlRight;
CTRL_t piLeft, piRight;

// Software I2C
uint8_t wireRx[MEMDEPTH_WIRE];
uint8_t wireTx[MEMDEPTH_WIRE];
QUEUE_t wireRxQueue, wireTxQueue;
WIRE_t wire;

// UART input
char uart3In, uart1In, uart2In;

// Recording
char recordBuf1[MEMDEPTH_RECORD];
char recordBuf2[MEMDEPTH_RECORD];
QUEUE_t record1, record2;
Recordmode recordmode = RECORD_RPM;
bool recordDone = false;
uint32_t records = 0;
uint32_t recordIt = 0;

// Compass and GPS
char kvhBuf[MEMDEPTH_KVH];
char gpsBuf[MEMDEPTH_GPS];
QUEUE_t kvhQueue;
QUEUE_t gpsQueue;

// General data containers
NMEA_InputData_t kvhInput;
MovingAvg_t kvhFilter;
int kvhFilterBuf[MEMDEPTH_KVH_MOVINGAVG];

NMEA_InputData_t gpsInput;

GPSData_t gpsData;

uint32_t adcValues[ADC_SAMPLES];
bool tgl = false;

TERMINALMODE_t terminalMode;

EMERGENCY_t emergencyStop;
bool shit;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * @brief Toggle SPI chipselect for debugging.
 */
void CSTGL()
{
	tgl = !tgl;
	if(tgl)
	{
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
	}
	else
	{
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	}
}

/*
 * @brief Disable IRQs
 */
void disableIRQ()
{
	__disable_irq();
}

/*
 * @brief Enable IRQs and restart UART
 */
void enableIRQ()
{
	__enable_irq();
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1In, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2In, 1);
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart3In, 1);
}

/*
 * @brief Send message to host
 * @param message Message payload
 * @prarm len Message length
 */
void INTERFACE_SendMessage(char* message, int len)
{
	if(!TERMINALMODE_IsEnabled(&terminalMode))
	{
		HAL_UART_Transmit(&huart3, (uint8_t*)message, (uint16_t)len, 1000);
	}
}

/*
 * @brief UART ISR
 * @brief Pushes data to corresponding dataqueues depending on operation mode.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	disableIRQ();
	if(huart->Instance == huart1.Instance) // GPS receiver
	{
		if(!QUEUE_IsFull(&gpsQueue))
		{
			QUEUE_Push(uart1In, &gpsQueue);
		}

		if(TERMINALMODE_IsEnabled(&terminalMode))
		{
			if(!QUEUE_IsFull(&record2))
			{
				QUEUE_Push(uart1In, &record2);
			}
		}
	}
	else if(huart->Instance == huart2.Instance) // Compass
	{
		if(!QUEUE_IsFull(&kvhQueue))
		{
			QUEUE_Push(uart2In, &kvhQueue);
		}

		if(TERMINALMODE_IsEnabled(&terminalMode))
		{
			if(!QUEUE_IsFull(&record2))
			{
				QUEUE_Push(uart2In, &record2);
			}
		}
	}
	else if(huart->Instance == huart3.Instance) // Host communication
	{
		if(!QUEUE_IsFull(&interfaceQueue))
		{
			QUEUE_Push(uart3In, &interfaceQueue);
		}

		if(TERMINALMODE_IsEnabled(&terminalMode))
		{
			if(!QUEUE_IsFull(&record1))
			{
				QUEUE_Push(uart3In, &record1);
			}
		}
	}
	enableIRQ();
}

/**
 * @brief Process recording activities and send data to host if demanded.
 * @brief Disables and reenables IRQs
 */
void RECORD_Process()
{
	char msgBuf[ROBOT_MESSAGE_MAXLEN];
	int len;
	int instruction;
	if(RECORD_RPM == recordmode || RECORD_CURRENT == recordmode) // RPM or Current
	{
		instruction = ROBOT_INSTRUCTION_RECORDRPM;
		if(RPMCTRL_IsRecordDone(&ctrlLeft) && RPMCTRL_IsRecordDone(&ctrlRight))
		{
			disableIRQ();
			RPMCTRL_StopRecord(&ctrlLeft);
			RPMCTRL_StopRecord(&ctrlRight);
			while(!QUEUE_IsEmpty(&record1) || !QUEUE_IsEmpty(&record2))
			{	  if(!QUEUE_IsEmpty(&kvhQueue))
			  {
				  NMEA_Parse(QUEUE_Pop(&kvhQueue), &kvhInput);
			  }
				uint16_t val1 = 10;
				uint16_t val2 = 10;
				if(!QUEUE_IsEmpty(&record1))
				{
					val1 = QUEUE_Pop16(&record1);
				}
				if(!QUEUE_IsEmpty(&record2))
				{
					val2 = QUEUE_Pop16(&record2);
				}
				len = INTERFACE_CreateMessage16(msgBuf, ROBOT_INSTRUCTION_RECORDRPM, val1, val2);
				INTERFACE_SendMessage(msgBuf, len);
			}
			len = INTERFACE_CreateMessage16(msgBuf, instruction, 0, 0);
			INTERFACE_SendMessage(msgBuf, len);
			INTERFACE_ResetPeriodics();
			enableIRQ();
		}
	}
	else if(RECORD_HEADING == recordmode && recordDone) // Compass heading
	{
		uint16_t val1 = 0;
		uint16_t val2 = 0;
		instruction = ROBOT_INSTRUCTION_RECORDHEADING;
		disableIRQ();
		while(!QUEUE_IsEmpty(&record1))
		{
			val1 = 362;
			val2 = 362;
			if(!QUEUE_IsEmpty(&record1))
			{
				val1 = QUEUE_Pop16(&record1);
			}
			if(!QUEUE_IsEmpty(&record1))
			{
				val2 = QUEUE_Pop16(&record1);
			}
			len = INTERFACE_CreateMessage16(msgBuf, instruction, val1, val2);
			INTERFACE_SendMessage(msgBuf, len);
		}
		while(!QUEUE_IsEmpty(&record2))
		{
			val1 = 362;
			val2 = 362;
			if(!QUEUE_IsEmpty(&record2))
			{
				val1 = QUEUE_Pop16(&record2);
			}
			if(!QUEUE_IsEmpty(&record2))
			{
				val2 = QUEUE_Pop16(&record2);
			}
			len = INTERFACE_CreateMessage16(msgBuf, instruction, val1, val2);
			INTERFACE_SendMessage(msgBuf, len);
		}
		len = INTERFACE_CreateMessage16(msgBuf, instruction, 362, 362);
		INTERFACE_SendMessage(msgBuf, len);
		recordmode = RECORD_RPM;
		recordDone = false;
		INTERFACE_ResetPeriodics();
		enableIRQ();
	}
}

/*
 * @brief Callback for host data processing
 * @param dat Received data
 */
void INTERFACE_Process(INTERFACE_Data_t* dat)
{
	disableIRQ();
	switch(dat->instruction)
	{
	case ROBOT_INSTRUCTION_DUTYCYCLE:
		RPMCTRL_Ignore(&ctrlLeft, true, &piLeft);
		RPMCTRL_Ignore(&ctrlRight, true, &piRight);
		RPMCTRL_SetDest(&ctrlLeft, (int)((short)dat->val1), &piLeft);
		RPMCTRL_SetDest(&ctrlRight, (int)((short)dat->val2), &piRight);
		break;
	case ROBOT_INSTRUCTION_RPM:
		RPMCTRL_Ignore(&ctrlLeft, false, &piLeft);
		RPMCTRL_Ignore(&ctrlRight, false, &piRight);
		RPMCTRL_SetDest(&ctrlLeft, (int)((short)dat->val1), &piLeft);
		RPMCTRL_SetDest(&ctrlRight, (int)((short)dat->val2), &piRight);
		break;
	case ROBOT_INSTRUCTION_LED:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (GPIO_PinState)dat->val2);
		break;
	case ROBOT_INSTRUCTION_RECORDRPM:
		recordmode = RECORD_RPM;
		RPMCTRL_StartRecord(&ctrlLeft, (uint32_t)dat->val1, &record1, false);
		RPMCTRL_StartRecord(&ctrlRight, (uint32_t)dat->val2, &record2, false);
		break;
	case ROBOT_INSTRUCTION_RECORDCURRENT:
		recordmode = RECORD_CURRENT;
		RPMCTRL_StartRecord(&ctrlLeft, (uint32_t)dat->val1, &record1, true);
		RPMCTRL_StartRecord(&ctrlRight, (uint32_t)dat->val2, &record2, true);
		break;
	case ROBOT_INSTRUCTION_LEFT_KP:
		CTRL_Init(piLeft.Ka, float_2x16(dat->val1, dat->val2), piLeft.Tn, piLeft.Td, piLeft.T, piLeft.Lp, piLeft.maxIn, piLeft.maxOut, &piLeft);
		break;
	case ROBOT_INSTRUCTION_LEFT_TN:
		CTRL_Init(piLeft.Ka, piLeft.Kp, float_2x16(dat->val1, dat->val2), piLeft.Td, piLeft.T, piLeft.Lp, piLeft.maxIn, piLeft.maxOut, &piLeft);
		break;
	case ROBOT_INSTRUCTION_LEFT_TD:
		CTRL_Init(piLeft.Ka, piLeft.Kp, piLeft.Tn, float_2x16(dat->val1, dat->val2), piLeft.T, piLeft.Lp, piLeft.maxIn, piLeft.maxOut, &piLeft);
		break;
	case ROBOT_INSTRUCTION_LEFT_KA:
		CTRL_Init(float_2x16(dat->val1, dat->val2), piLeft.Kp, piLeft.Tn, piLeft.Td, piLeft.T, piLeft.Lp, piLeft.maxIn, piLeft.maxOut, &piLeft);
		break;
	case ROBOT_INSTRUCTION_RIGHT_KP:
		CTRL_Init(piRight.Ka, float_2x16(dat->val1, dat->val2), piRight.Tn, piRight.Td, piRight.T, piRight.Lp, piRight.maxIn, piRight.maxOut, &piRight);
		break;
	case ROBOT_INSTRUCTION_RIGHT_TN:
		CTRL_Init(piRight.Ka, piRight.Kp, float_2x16(dat->val1, dat->val2), piRight.Td, piRight.T, piRight.Lp, piRight.maxIn, piRight.maxOut, &piRight);
		break;
	case ROBOT_INSTRUCTION_RIGHT_TD:
		CTRL_Init(piRight.Ka, piRight.Kp, piRight.Tn, float_2x16(dat->val1, dat->val2), piRight.T, piRight.Lp, piRight.maxIn, piRight.maxOut, &piRight);
		break;
	case ROBOT_INSTRUCTION_RIGHT_KA:
		CTRL_Init(float_2x16(dat->val1, dat->val2), piRight.Kp, piRight.Tn, piRight.Td, piRight.T, piRight.Lp, piRight.maxIn, piRight.maxOut, &piRight);
		break;
	case ROBOT_INSTRUCTION_RPM_LOWPASS:
		CTRL_Init(piLeft.Ka, piLeft.Kp, piLeft.Tn, piLeft.Td, piLeft.T, float_2x16(dat->val1, dat->val2), piLeft.maxIn, piLeft.maxOut, &piLeft);
		CTRL_Init(piRight.Ka, piRight.Kp, piRight.Tn, piRight.Td, piRight.T, float_2x16(dat->val1, dat->val2), piRight.maxIn, piRight.maxOut, &piRight);
		break;
	case ROBOT_INSTRUCTION_RECORDHEADING:
		records = uint32_2x16(dat->val1, dat->val2);
		recordDone = false;
		recordIt = 0;
		QUEUE_Reset(&record1);
		QUEUE_Reset(&record2);
		recordmode = RECORD_HEADING;
		break;
	case ROBOT_PERIODIC_DATE:
	case ROBOT_PERIODIC_ENCODERS:
	case ROBOT_PERIODIC_HEADING:
	case ROBOT_PERIODIC_LATLON:
	case ROBOT_PERIODIC_TIME:
		INTERFACE_SetPeriod((uint8_t)dat->instruction, dat->val2);
		break;
	case ROBOT_INSTRUCTION_TERMINALMODE:
		TERMINALMODE_Config(&terminalMode, dat->val2);
		INTERFACE_EnableCRC(!(TERMINAL_NOCS == terminalMode.input));
	case ROBOT_INSTRUCTION_LEFT_CORRFAC_SHORT:
		ctrlLeft.corrfacShort = float_2x16(dat->val1, dat->val2);
		break;
	case ROBOT_INSTRUCTION_LEFT_CORRFAC_LONG:
		ctrlLeft.corrfacLong = float_2x16(dat->val1, dat->val2);
		break;
	case ROBOT_INSTRUCTION_RIGHT_CORRFAC_SHORT:
		ctrlRight.corrfacShort = float_2x16(dat->val1, dat->val2);
		break;
	case ROBOT_INSTRUCTION_RIGHT_CORRFAC_LONG:
		ctrlRight.corrfacLong = float_2x16(dat->val1, dat->val2);
		break;
	case ROBOT_INSTRUCTION_RESET:
		ctrlLeft.eternalTicks = 0;
		ctrlRight.eternalTicks = 0;
		break;
	case ROBOT_INSTRUCTION_BALLSHIT:
		shit = true;
		break;
	default: break;
	}
	enableIRQ();
}

/*
 * @brief Get Systick with 100 us resolution.
 * @brief Can overflow.
 * @return Time in us
 */
uint32_t getTick()
{
	return htim2.Instance->CNT * 100;
}

/**
 * @brief Interrupt for first encoder
 */
void ISR_ENCODER1(void)
{
	disableIRQ();
	static bool isLow = false;
	static uint32_t tick = 0;
	tick = getTick();
	isLow = !(GPIO_PIN_SET == HAL_GPIO_ReadPin(ENCODER1_GPIO_Port, ENCODER1_Pin));
	RPMCTRL_Tick(&ctrlLeft, tick, isLow);
	ODOMETRY_Update(&odom, ctrlLeft.rpm, ctrlRight.rpm, tick);
	enableIRQ();
}

/*
 * @brief Interrupt for second encoder
 */
void ISR_ENCODER0(void)
{
	disableIRQ();
	static bool isLow = false;
	static uint32_t tick = 0;
	tick = getTick();
	isLow = !(GPIO_PIN_SET == HAL_GPIO_ReadPin(ENCODER0_GPIO_Port, ENCODER0_Pin));
	RPMCTRL_Tick(&ctrlRight, tick, isLow);
	ODOMETRY_Update(&odom, ctrlLeft.rpm, ctrlRight.rpm, tick);
	enableIRQ();
}

/*
 * @brief 10 Hz timer interrupt for control loops
 */
void ISR_10Hz(void)
{
	disableIRQ();
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
	RPMCTRL_Update(&ctrlLeft, &piLeft);
	RPMCTRL_Update(&ctrlRight, &piRight);
	Powertrain_SetDutycycles(ctrlLeft.dutyCycle, ctrlRight.dutyCycle);
	ADCFILTER_Process(adcValues, ADC_SAMPLES);
	RPMCTRL_SetAvgTorque(&ctrlLeft, (uint16_t)ADCFILTER_Values[ADC_IS12]);
	RPMCTRL_SetAvgTorque(&ctrlRight, (uint16_t)ADCFILTER_Values[ADC_IS34]);
	HAL_ADC_Start_DMA(&hadc1, adcValues, ADC_SAMPLES);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	enableIRQ();
}

/*
 * @brief Emergency button event
 */
void ISR_EMSTOP(void)
{
	static bool isLow = false;
	isLow = (GPIO_PIN_SET == HAL_GPIO_ReadPin(EMSTOP_GPIO_Port, EMSTOP_Pin));

	emergencyStop.isEvent = true;
	emergencyStop.isEmergency = isLow;

	Powertrain_SetDutycycles(0, 0);
	RPMCTRL_SetDest(&ctrlLeft, 0, &piLeft);
	RPMCTRL_SetDest(&ctrlRight, 0, &piRight);
}

/*
 * @brief NMEA HDT data callback from compass
 * @param dat HDT data
 */
void NMEA_CallbackHDT(NMEA_HDTData_t* dat)
{
	MOVINGAVG_AddSample(&kvhFilter, (int)dat->heading);
	MOVINGAVG_Calculate(&kvhFilter);
	if(kvhFilter.output > 180)
	{
		kvhFilter.output -= 360;
	}

	if(RECORD_HEADING == recordmode)
	{
		if(recordIt < records)
		{
			if(!QUEUE_IsFull(&record1))
			{
				QUEUE_Push16((uint16_t)kvhFilter.output, &record1);
			}
			else if(!QUEUE_IsFull(&record2))
			{
				QUEUE_Push16((uint16_t)kvhFilter.output, &record2);
			}
			recordIt ++;
		}
		else
		{
			recordDone = true;
		}
	}
}

/*
 * @brief NMEA RMC data callback from GPS
 * @param dat RMC data
 */
void NMEA_CallbackRMC(NMEA_RMCData_t* dat)
{
	gpsData.latitude = dat->latitude;
	gpsData.longitude = dat->longitude;
	gpsData.day = dat->day;
	gpsData.month = dat->month;
	gpsData.year = dat->year;
	gpsData.hours = dat->hours;
	gpsData.minutes = dat->minutes;
	gpsData.seconds = dat->seconds;
}

void ISR_DMA(void){}

/**
 * @brief Process periodical messages and send them to host.
 * @param p Required periodic
 */
void INTERFACE_PeriodicRequired(INTERFACE_Periodic_t* p)
{
	char msgBuf[ROBOT_MESSAGE_MAXLEN];
	int len;
	disableIRQ();
	switch(p->id)
	{
	case ROBOT_PERIODIC_HEADING:
		len = INTERFACE_CreateMessage16(msgBuf, p->id, 0, (uint16_t)kvhFilter.output);
		INTERFACE_SendMessage(msgBuf, len);
		break;
	case ROBOT_PERIODIC_LATLON:
		len = INTERFACE_CreateMessage32(msgBuf, p->id, 0xffffffff);
		INTERFACE_SendMessage(msgBuf, len);
		len = INTERFACE_CreateMessageFloat(msgBuf, p->id, gpsData.latitude);
		INTERFACE_SendMessage(msgBuf, len);
		len = INTERFACE_CreateMessageFloat(msgBuf, p->id, gpsData.longitude);
		INTERFACE_SendMessage(msgBuf, len);
		break;
	case ROBOT_PERIODIC_TIME:
		len = INTERFACE_CreateMessage8(msgBuf,
				p->id,
				0,
				(uint8_t)gpsData.hours,
				(uint8_t)gpsData.minutes,
				(uint8_t)gpsData.seconds);
		INTERFACE_SendMessage(msgBuf, len);
		break;
	case ROBOT_PERIODIC_DATE:
		len = INTERFACE_CreateMessage8(msgBuf,
				p->id,
				0,
				(uint8_t)gpsData.year,
				(uint8_t)gpsData.month,
				(uint8_t)gpsData.day);
		INTERFACE_SendMessage(msgBuf, len);
		break;
	case ROBOT_PERIODIC_ENCODERS:
		len = INTERFACE_CreateMessage32(msgBuf, p->id, 0xffffffff);
		INTERFACE_SendMessage(msgBuf, len);
		len = INTERFACE_CreateMessage32(msgBuf, p->id, HAL_GetTick());
		INTERFACE_SendMessage(msgBuf, len);
		len = INTERFACE_CreateMessage32(msgBuf, p->id, (uint32_t)((int)ctrlLeft.eternalTicks));
		INTERFACE_SendMessage(msgBuf, len);
		len = INTERFACE_CreateMessage32(msgBuf, p->id, (uint32_t)((int)ctrlRight.eternalTicks));
		INTERFACE_SendMessage(msgBuf, len);
	case ROBOT_PERIODIC_ODOM:
		len = INTERFACE_CreateMessage32(msgBuf, p->id, 0xffffffff);
		INTERFACE_SendMessage(msgBuf, len);
		INTERFACE_CreateMessageFloat(msgBuf, p->id, odom.x);
		INTERFACE_SendMessage(msgBuf, len);
		INTERFACE_CreateMessageFloat(msgBuf, p->id, odom.y);
		INTERFACE_SendMessage(msgBuf, len);
		INTERFACE_CreateMessageFloat(msgBuf, p->id, odom.o);
		INTERFACE_SendMessage(msgBuf, len);
		INTERFACE_CreateMessageFloat(msgBuf, p->id, odom.dx);
		INTERFACE_SendMessage(msgBuf, len);
		INTERFACE_CreateMessageFloat(msgBuf, p->id, odom.dy);
		INTERFACE_SendMessage(msgBuf, len);
		INTERFACE_CreateMessageFloat(msgBuf, p->id, odom.w);
		INTERFACE_SendMessage(msgBuf, len);
	default: break;
	}
	enableIRQ();
}

/*
 * @brief Process terminal modes used for direct UART sensor pass-through.
 */
void TERMINAL_Process()
{
	  if(TERMINALMODE_IsEnabled(&terminalMode))
	  {
		  while(!QUEUE_IsEmpty(&record1))
		  {
			  char c = QUEUE_Pop(&record1);

			  if(TERMINALMODE_IsEnabled(&terminalMode))
			  {
			  	switch(terminalMode.input)
			  	{
			  		case TERMINAL_UART1: HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, 100); break;
			  		case TERMINAL_UART2: HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 100); break;
			  		default: break;
			  	}
			  }
		  }
		  while(!QUEUE_IsEmpty(&record2))
		  {
			  char c = QUEUE_Pop(&record2);
			  HAL_UART_Transmit(&huart3, (uint8_t*)&c, 1, 1000);
		  }
	  }
}

/*
 * @brief Process GPS data from receive buffer
 */
void GPS_Process()
{
	  while(!QUEUE_IsEmpty(&gpsQueue))
	  {
		  NMEA_Parse(QUEUE_Pop(&gpsQueue), &gpsInput);
	  }
}

/*
 * @brief Process compass data from receive buffer
 */
void KVH_Process()
{
	  while(!QUEUE_IsEmpty(&kvhQueue))
	  {
		  NMEA_Parse(QUEUE_Pop(&kvhQueue), &kvhInput);
	  }
}

/*
 * @brief Process emergency events
 */
void EMERGENCY_Process()
{
	if(emergencyStop.isEvent)
	{
		emergencyStop.isEvent = false;
		char buf[INTERFACE_MSG_LEN + 1];
		int len = INTERFACE_CreateMessage16(buf, ROBOT_INSTRUCTION_EMSTOP, 0, emergencyStop.isEmergency);
		INTERFACE_SendMessage(buf, len);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Init recording
  QUEUE_Init(&record1, recordBuf1, MEMDEPTH_RECORD);
  QUEUE_Init(&record2, recordBuf2, MEMDEPTH_RECORD);

  // Init host communication
  QUEUE_Init(&interfaceQueue, interfaceBuf, MEMDEPTH_INTERFACE);
  INTERFACE_Init();
  INTERFACE_AddPeriodic(ROBOT_PERIODIC_LATLON, ROBOT_PERIOD_LATLON);
  INTERFACE_AddPeriodic(ROBOT_PERIODIC_TIME, ROBOT_PERIOD_TIME);
  INTERFACE_AddPeriodic(ROBOT_PERIODIC_DATE, ROBOT_PERIOD_DATE);
  INTERFACE_AddPeriodic(ROBOT_PERIODIC_HEADING, ROBOT_PERIOD_HEADING);
  INTERFACE_AddPeriodic(ROBOT_PERIODIC_ENCODERS, ROBOT_PERIOD_ENCODERS);
  INTERFACE_AddPeriodic(ROBOT_PERIODIC_ODOM, ROBOT_PERIOD_ODOM);

  // Init RPM control
  CTRL_Init(ROBOT_LEFT_KA, ROBOT_LEFT_KP, ROBOT_LEFT_TN, ROBOT_LEFT_TD, ROBOT_RPMCTRL_T, ROBOT_LEFT_LP, ROBOT_RPM_MAX, ROBOT_DUTYCYCLE_MAX, &piLeft);
  CTRL_Init(ROBOT_RIGHT_KA, ROBOT_RIGHT_KP, ROBOT_RIGHT_TN, ROBOT_RIGHT_TD, ROBOT_RPMCTRL_T, ROBOT_RIGHT_LP, ROBOT_RPM_MAX, ROBOT_DUTYCYCLE_MAX, &piRight);
  RPMCTRL_Init(ROBOT_ENCODER_STEPS, ROBOT_LEFT_CORRFACSHORT, ROBOT_LEFT_CORRFACLONG, &ctrlLeft);
  RPMCTRL_Init(ROBOT_ENCODER_STEPS, ROBOT_RIGHT_CORRFACSHORT, ROBOT_RIGHT_CORRFACLONG, &ctrlRight);
  ODOMETRY_Init(ROBOT_ENCODER_STEPS, ROBOT_WHEELRADIUS_M, ROBOT_WHEELWIDTH_M, &odom);

  // Init UART sensors
  QUEUE_Init(&kvhQueue, kvhBuf, MEMDEPTH_KVH);
  QUEUE_Init(&gpsQueue, gpsBuf, MEMDEPTH_GPS);

  MOVINGAVG_Init(&kvhFilter, kvhFilterBuf, MEMDEPTH_KVH_MOVINGAVG);
  NMEA_Init(&kvhInput);

  NMEA_Init(&gpsInput);

  TERMINALMODE_Config(&terminalMode, TERMINAL_DISABLE);

  emergencyStop.isEmergency = false;
  emergencyStop.isEvent = false;
  __enable_irq();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  Powertrain_Init();
  Ballshitter_Init();
  //KVH_Init();

  // Init software I2C
  QUEUE_Init(&wireRxQueue, (char*)wireRx, 20);
  QUEUE_Init(&wireTxQueue, (char*)wireTx, 20);
  WIRE_Init(&wire,
		  SDA_GPIO_Port, SDA_Pin,
		  SCL_GPIO_Port, SCL_Pin,
		  400,
		  &wireRxQueue, &wireTxQueue);
  WIRE_Begin(&wire);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  disableIRQ();
//	  __disable_irq();
	  INTERFACE_Parse(&interfaceQueue);
	  RECORD_Process();
	  INTERFACE_ProcessPeriodics();
	  TERMINAL_Process();
	  KVH_Process();
//	  GPS_Process();
	  EMERGENCY_Process();
	  Ballshitter_Drop();
//	  	__enable_irq();

//	  	HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1In, 1);
//	  	HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2In, 1);
//	  	HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart3In, 1);
//	    HAL_TIM_Base_Start_IT(&htim2);
//	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//	    HAL_TIM_Base_Start_IT(&htim4);
//	  HAL_Delay(200);
//	  enableIRQ();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
//  hadc1.Init.ContinuousConvMode = ENABLE;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  HAL_ADC_Start_DMA(&hadc1, adcValues, ADC_SAMPLES);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 21;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7250;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6550;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
//	HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1In, 1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 4800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2In, 1);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart3In, 1);
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|INH12_Pin|INH34_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCL_Pin|SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin INH12_Pin INH34_Pin */
  GPIO_InitStruct.Pin = LED_Pin|INH12_Pin|INH34_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EMSTOP_Pin ENCODER0_Pin */
  GPIO_InitStruct.Pin = EMSTOP_Pin|ENCODER0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER1_Pin */
  GPIO_InitStruct.Pin = ENCODER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * @brief Init powertrain.
 */
void Powertrain_Init()
{
	Powertrain_EnableMotors(1);
	Powertrain_SetDutycycles(0, 0);
}

/*
 * @brief Enable motors
 * @param enableState true for enabling
 */
void Powertrain_EnableMotors(int enableState)
{
	HAL_GPIO_WritePin(INH12_GPIO_Port, INH12_Pin, enableState);
	HAL_GPIO_WritePin(INH34_GPIO_Port, INH34_Pin, enableState);
}

/*
 * @brief Set motor dutycycles
 * @param left dutycycle from -100 to 100
 * @param right dutycycle from -100 to 100
 */
void Powertrain_SetDutycycles(int left, int right)
{
	static int lastLeft = 0;
	static int lastRight = 0;

	if(left > 100)
	{
		left = 100;
	}
	if(left < -100)
	{
		left = -100;
	}

	if(right > 100)
	{
		right = 100;
	}
	if(right < -100)
	{
		right = -100;
	}

	left *= 10;
	right *= 10;

//	if(left != lastLeft)
//	{
		if(left < 0)
		{
			left = -left;
			htim3.Instance->CCR3 = left;
			htim3.Instance->CCR4 = 0;
		}
		else
		{
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = left;
		}
		lastLeft = left;
//	}

//	if(right != lastRight)
//	{
		if(right < 0)
		{
			right = -right;
			htim3.Instance->CCR1 = right;
			htim3.Instance->CCR2 = 0;
		}
		else
		{
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR2 = right;
		}
//		lastRight = right;
//	}
}

void Servo_Set(int angle)
{
	if(angle < 0) angle = 0;
	else if(angle > 180) angle = 180;

	htim1.Instance->CCR1 = SERVO_OFFSET + (angle * SERVO_RANGE) / 180;
}

void Ballshitter_Init()
{
	shit = false;
	Servo_Set(180);
	HAL_Delay(BALLSHITTER_TIME);
}

void Ballshitter_Drop()
{
	static uint32_t time = 0;
	static bool init = false;
	uint32_t current = HAL_GetTick();
	if(!shit || !init)
	{
		init = true;
		time = HAL_GetTick();
	}
	else if(shit)
	{
		uint32_t delta = current - time;
		if(delta < BALLSHITTER_TIME)
		{
			Servo_Set(180 - (180 * delta) / BALLSHITTER_TIME);
		}
		else if(delta < 2 * BALLSHITTER_TIME)
		{
			Servo_Set((180 * (delta - BALLSHITTER_TIME)) / BALLSHITTER_TIME);
		}
		shit = delta < 3 * BALLSHITTER_TIME;
	}
}

/*
 * @brief Init KVH-C100 compass.
 */
void KVH_Init()
{

	  char kvhConfigNmeaStr[5] = { '=', 't' , ',', '0', '\r' }; // NMEA protocol
	  char kvhConfigSpeedStr[8] = { '=', 'r', ',' , '6' , '0', '0', '\r' }; // 1Hz
	  char kvhConfigUnitStr[5] = { '=', 'i' , ',', 'd', '\r' }; // degrees as unit
	  char kvhDampingTypeStr[7] = { '=',  'd', 't', '0', ',', '3', '\r' }; // double IIR
	  char kvhDampingTimeStr[6] = { '=',  'd', '0', ',', '9', '\r' }; // 24 secs
	  char kvhInitStr[3] = { 's', '\r' }; // start

	  HAL_UART_Transmit(&huart2, (uint8_t*)kvhConfigNmeaStr, 5, 1000);
	  HAL_Delay(200);
	  HAL_UART_Transmit(&huart2, (uint8_t*)kvhConfigUnitStr, 5, 1000);
	  HAL_Delay(200);
	  HAL_UART_Transmit(&huart2, (uint8_t*)kvhConfigSpeedStr, 7, 1000);
	  HAL_Delay(200);
	  HAL_UART_Transmit(&huart2, (uint8_t*)kvhDampingTypeStr, 7, 1000);
	  HAL_Delay(200);
	  HAL_UART_Transmit(&huart2, (uint8_t*)kvhDampingTimeStr, 6, 1000);
	  HAL_Delay(200);
	  HAL_UART_Transmit(&huart2, (uint8_t*)kvhInitStr, 2, 1000);
	  HAL_Delay(200);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
