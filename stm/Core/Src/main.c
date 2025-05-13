/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "QueueManager.h"
#include "pid.h"
#include "ICM20948.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <float.h>
#include <stdlib.h>
#include <Appl_Timer.h>
#include <HCSR04_cfg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
////////////////////COMMAND TABLE////////////////////
#define MOVE_FORWARD          1
#define MOVE_BACKWARD         2
#define TURN_FORWARD_LEFT     3
#define TURN_FORWARD_RIGHT    4
#define TURN_BACKWARD_LEFT    5
#define TURN_BACKWARD_RIGHT   6
#define OFFSET_FORWARD_LEFT   7
#define OFFSET_FORWARD_RIGHT  8
#define OFFSET_BACKWARD_LEFT  9
#define OFFSET_BACKWARD_RIGHT 10
#define STOP                  11
#define SNAP				  12


////////////////////RPI RECEIVER////////////////////
#define RXBUFFER_SIZE 8

////////////////////ENCODER DEFINES////////////////////

#define REAR_WHEEL_RADIUS_CM 3.3
#define REAR_WHEEL_CIRCUMFERENCE (2.0 * 3.142 * REAR_WHEEL_RADIUS_CM)
#define ENCODER_PULSES_PER_ROTATION 1320 // 11*4*30
#define PULSE_PER_DISTANCE (ENCODER_PULSES_PER_ROTATION / REAR_WHEEL_CIRCUMFERENCE)
#define LOAD_TORQUE_RPM 266
#define MAX_ENCODER_SPEED ((ENCODER_PULSES_PER_ROTATION * LOAD_TORQUE_RPM) / 60)
#define MAX_MOTOR_PWM 3500

////////////////////SERVO MOTOR DEFINES////////////////////
#define CENTER_POS_PWM 151
#define LEFT_DELTA 47  // max 80
#define RIGHT_DELTA 70 // max 80
#define LEFT_OFFSET_DELTA 15  // max 80
#define RIGHT_OFFSET_DELTA 15 // max 80
#define STEERING_BUFFER_TIME 250
#define LEFTTURN_POS_PWM (CENTER_POS_PWM - LEFT_DELTA)
#define RIGHTTURN_POS_PWM (CENTER_POS_PWM + RIGHT_DELTA)
#define LEFTOFF_POS_PWM (CENTER_POS_PWM - LEFT_OFFSET_DELTA)
#define RIGHTOFF_POS_PWM (CENTER_POS_PWM + RIGHT_OFFSET_DELTA)

// Gyroscope Defines///
#define NUM_CALIBRATION_SAMPLES 500
#define GYRO_SENSITIVITY_SCALE_FACTOR_2000DPS 16.4f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Command_Task */
osThreadId_t Command_TaskHandle;
const osThreadAttr_t Command_Task_attributes = {
		.name = "Command_Task",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
		.name = "IMU_Task",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PID_Task */
osThreadId_t PID_TaskHandle;
const osThreadAttr_t PID_Task_attributes = {
		.name = "PID_Task",
		.stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
		.name = "UltrasonicTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for HeadingPID */
osThreadId_t HeadingPIDHandle;
const osThreadAttr_t HeadingPID_attributes = {
		.name = "HeadingPID",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRSensorTask */
osThreadId_t IRSensorTaskHandle;
const osThreadAttr_t IRSensorTask_attributes = {
		.name = "IRSensorTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
char debugBuffer[100];  // Buffer for debug messages
uint8_t OLEDBuffer[32]; // store oled text
uint8_t rxBuffer[RXBUFFER_SIZE + 2]; // store command from rpi
volatile uint16_t speedCmd = 0; // in count/seconds
volatile uint16_t targetDistance = 0, targetAngle = 0; // total pulse to travel
bool commandStatus = false; // check if commandTask is running a command

volatile float heading = 0.0f;        // Heading in [0, 360)
static int16_t gyroZ_offset = 0;
uint8_t readGyroZData[2];
uint8_t Hdirection = 0;


float leftSpeed; // encodetask
float rightSpeed;
volatile uint32_t leftTotalCount = 0;
volatile uint32_t rightTotalCount = 0;

volatile uint16_t leftOutputPWM = 0;
volatile uint16_t rightOutputPWM = 0;

pid_type_def forward_pid, backward_pid;

fp32 pid_param_forward[3] = { 3.5, 0.05, 0.4};
fp32 pid_param_backward[3] = { 3.7, 0.1, 0.22};
fp32 pid_param_turn[3] = { 10, 0.0, 0.0 }; // used for PID steering (not as accurate as Gyro but quicker)
fp32 max_out = 4000;
fp32 max_out_turn = 4500;
fp32 max_iout = 3000;

float error_sum = 0.01;
float last_error = 0.0f;
volatile float targetHeading = 0.0f;
uint8_t cmdID = 0;

osMutexId_t headingMutex;
const osMutexAttr_t headingMutex_attributes = {
		.name = "headingMutex"
};
bool resetEncoderFlag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void commandTask(void *argument);
void imuTask(void *argument);
void pidTask(void *argument);
void Ultrasonic_Task(void *argument);
void Heading_PID(void *argument);
void IRSensor_Task(void *argument);

/* USER CODE BEGIN PFP */
void Start_RPI_Comm(void);
void SendMsg_RPI(uint8_t *pdata , uint32_t u32Len);
void turnLeft();
void turnRight();
void offsetLeft();
void offsetRight();
void faceFront();
void setLeftPWM(uint16_t dutyCycle);
void setRightPWM(uint16_t dutyCycle);
void setMotorForward();
void setMotorBackward();
void motorStop();
void moveMotor(int direction, uint16_t speed);
//void distanceTracker();
uint32_t getEncoderDelta(uint32_t count1, uint32_t count2, TIM_HandleTypeDef *htim);
//osSemaphoreId_t commandSemaphore;
//const osSemaphoreAttr_t commandSemaphore_attributes = {
//		.name = "commandSemaphore"
//};


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_TIM8_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	OLED_Init();
	Start_RPI_Comm();
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // Left Motor PWM
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // Right Motor PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Servo Motor PWM
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Left Motor Encoder
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Right Motor Encoder

	ICM20948_init(&hi2c1, 0, GYRO_FULL_SCALE_2000DPS);

	PID_init(&forward_pid, PID_POSITION, pid_param_forward, max_out, max_iout);
	PID_init(&backward_pid, PID_POSITION, pid_param_backward, max_out, max_iout);

	//commandSemaphore = osSemaphoreNew(1, 0, &commandSemaphore_attributes);
	osThreadNew(commandTask, NULL, &Command_Task_attributes);
	headingMutex = osMutexNew(&headingMutex_attributes);
	if (headingMutex == NULL) {
		// Handle mutex creation failure
		Error_Handler();
	}



	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of Command_Task */
	Command_TaskHandle = osThreadNew(commandTask, NULL, &Command_Task_attributes);

	/* creation of IMU_Task */
	IMU_TaskHandle = osThreadNew(imuTask, NULL, &IMU_Task_attributes);

	/* creation of PID_Task */
	PID_TaskHandle = osThreadNew(pidTask, NULL, &PID_Task_attributes);

	/* creation of UltrasonicTask */
	UltrasonicTaskHandle = osThreadNew(Ultrasonic_Task, NULL, &UltrasonicTask_attributes);

	/* creation of HeadingPID */
	HeadingPIDHandle = osThreadNew(Heading_PID, NULL, &HeadingPID_attributes);

	/* creation of IRSensorTask */
	IRSensorTaskHandle = osThreadNew(IRSensor_Task, NULL, &IRSensorTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 160;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

	/* USER CODE END TIM2_Init 2 */

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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 7199;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

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

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
			|LED3_Pin|CIN1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin CIN1_Pin */
	GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
			|LED3_Pin|CIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : AIN2_Pin AIN1_Pin */
	GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : CIN2_Pin */
	GPIO_InitStruct.Pin = CIN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CIN2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ECHO_PIN_Pin */
	GPIO_InitStruct.Pin = ECHO_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ECHO_PIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : HCSR04_TRIG_Pin */
	GPIO_InitStruct.Pin = HCSR04_TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HCSR04_TRIG_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
////////////////////RPI COMMS START////////////////////
void Start_RPI_Comm(void)
{
	HAL_UART_Receive_IT(&huart3, (uint8_t*) rxBuffer, RXBUFFER_SIZE);  //trigger interrupt after 8 byte
}
void SendMsg_RPI(uint8_t *pdata , uint32_t u32Len)
{
	HAL_UART_Transmit(&huart3, pdata , u32Len, 100);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint32_t u32Cmd = 0U;
	uint32_t u32speed = 0U, u32distance = 0U;
	char msgBuffer[50];

	if (huart->Instance == USART3)
	{
		char direction = rxBuffer[0];
		char speedData[3] = {0};
		char distanceData[4] = {0};

		strncpy(speedData, (char*)&rxBuffer[1], 2);
		speedData[2] = '\0';
		strncpy(distanceData, (char*)&rxBuffer[4], 3);
		distanceData[3] = '\0';
		u32speed = (uint32_t)atoi(speedData);  // Convert ASCII to integer
		u32distance = (uint32_t)atoi(distanceData);  // Convert ASCII to integer
		if (u32distance != 0 && u32distance <= 5) {
			u32distance = 5;
		}

		switch (direction)
		{
		case 'F':  u32Cmd = MOVE_FORWARD;          break;
		case 'B':  u32Cmd = MOVE_BACKWARD;         break;
		case 'L':  u32Cmd = TURN_FORWARD_LEFT;     break;
		case 'R':  u32Cmd = TURN_FORWARD_RIGHT;    break;
		case 'l':  u32Cmd = TURN_BACKWARD_LEFT;    break;
		case 'r':  u32Cmd = TURN_BACKWARD_RIGHT;   break;
		case 'O':  u32Cmd = OFFSET_FORWARD_LEFT;   break;
		case 'P':  u32Cmd = OFFSET_FORWARD_RIGHT;  break;
		case 'o':  u32Cmd = OFFSET_BACKWARD_LEFT;  break;
		case 'p':  u32Cmd = OFFSET_BACKWARD_RIGHT; break;
		case 'Z':  u32Cmd = STOP;                  break;
		case 'S':  u32Cmd = SNAP; 				 break;
		default:   u32Cmd = 0U;                    break;
		}

		if (u32Cmd != 0U)
		{
			u32Cmd = (u32Cmd * 100000) + (u32speed * 1000) + u32distance;
			snprintf(msgBuffer, sizeof(msgBuffer), "%lu\n", (unsigned long)u32Cmd);
			SendMsg_RPI((uint8_t*)msgBuffer, strlen(msgBuffer));
			if(PushToQueue_MTR_Cmd(&u32Cmd))
			{
				//SendMsg_RPI((uint8_t*)"ACK1\n", strlen("ACK1\n"));
			}
			else
			{
				// If the queue is full, send an error message
				SendMsg_RPI((uint8_t*)"QUEUE_ERR\n", strlen("QUEUE_ERR\n"));
			}
		}
		else
		{
			SendMsg_RPI((uint8_t*)"CMD_FORMAT_ERR\n", strlen("CMD_FORMAT_ERR\n"));
		}
	}
	//memset(rxBuffer, 0U, RXBUFFER_SIZE);
	Start_RPI_Comm();
}
////////////////////RPI COMMS END////////////////////

////////////////////MOTOR HELPER START////////////////////
void turnLeft() {
	htim1.Instance->CCR4 = LEFTTURN_POS_PWM;
	//robot_direction = 0;
}
void turnRight() {
	htim1.Instance->CCR4 = RIGHTTURN_POS_PWM;
	//robot_direction = 2;
}
void offsetLeft() {
	htim1.Instance->CCR4 = LEFTOFF_POS_PWM;
	//robot_direction = 0;
}
void offsetRight() {
	htim1.Instance->CCR4 = RIGHTOFF_POS_PWM;
	//robot_direction = 2;
}
void faceFront() {
	htim1.Instance->CCR4 = CENTER_POS_PWM; //CENTER_POS_PWM;
	//robot_direction = 1;
}
void setLeftPWM(uint16_t dutyCycle) {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, dutyCycle);
}
void setRightPWM(uint16_t dutyCycle) {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, dutyCycle);
}
void setMotorForward() {
	// set motor direction
	// ------- left motor
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	// ------- right motor
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
}
void setMotorBackward() {
	// set motor direction
	// ------- left motor
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
	// ------- right motor
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
}
void motorStop() {
	// ------- left motor
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
	setLeftPWM(0);
	// ------- right motor
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	setRightPWM(0);
}
void moveMotor(int direction, uint16_t speed) {
	if (direction == MOVE_FORWARD) { // Forward
		setMotorForward();
	} else if (direction == MOVE_BACKWARD){ // Backward
		setMotorBackward();
	}
	// Set PWM Compare
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed); // left motor
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, speed); // right motor
}
////////////////////MOTOR HELPER END////////////////////


void TriggerPulse(void)
{
	HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // Reset Timer
	while (__HAL_TIM_GET_COUNTER(&htim1) < 10);  // Wait 10µs
	HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
}



uint32_t MeasureEchoTime(void)
{
	uint32_t start_time = 0, stop_time = 0;

	// **Wait for ECHO to go HIGH (Start timing)**
	while (HAL_GPIO_ReadPin(ECHO_PIN_GPIO_Port, ECHO_PIN_Pin) == GPIO_PIN_RESET);
	start_time = __HAL_TIM_GET_COUNTER(&htim1);  // Get the start time in µs

	// **Wait for ECHO to go LOW (Stop timing)**
	while (HAL_GPIO_ReadPin(ECHO_PIN_GPIO_Port, ECHO_PIN_Pin) == GPIO_PIN_SET);
	stop_time = __HAL_TIM_GET_COUNTER(&htim1);  // Get the stop time in µs

	return (stop_time > start_time) ? (stop_time - start_time) : 0;
}

float CalculateDistance(uint32_t pulse_time)
{
	return (pulse_time * 0.0343f) / 2.0f;  // Convert to cm (divide by 2 for round trip)
}


////////////////////ENCODER START////////////////////

uint32_t getEncoderDelta(uint32_t count1, uint32_t count2,
		TIM_HandleTypeDef *htim) {
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
		if (count2 <= count1) {
			return count1 - count2;
		} else if ((count2 - count1) > 32000) {
			return (65535 - count2 + count1);
		}
		else {
			return (count2 - count1);
		}
	} else {
		if (count2 >= count1) {
			return count2 - count1;
		} else if ((count1 - count2) > 32000){
			return (65535 - count1 + count2);
		}
		else {
			return (count1 - count2);
		}

	}
}

void CalibrateGyroZOffset(I2C_HandleTypeDef *hi2c) {
	int16_t gyroZ_raw = 0;
	int16_t gyroZ_samples[NUM_CALIBRATION_SAMPLES];
	uint8_t gyroDataBuffer[2];  // Buffer for raw gyro data

	// **Step 1: Collect multiple stable samples**
	for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
		__Gyro_Read_Z(hi2c, gyroDataBuffer, gyroZ_raw);  // Uses the macro as per your main.h
		gyroZ_samples[i] = gyroZ_raw;
		osDelay(5);  // Small delay to reduce noise
	}

	// **Step 2: Sort samples to remove outliers (Optional but recommended)**
	for (int i = 0; i < NUM_CALIBRATION_SAMPLES - 1; i++) {
		for (int j = i + 1; j < NUM_CALIBRATION_SAMPLES; j++) {
			if (gyroZ_samples[i] > gyroZ_samples[j]) {
				int16_t temp = gyroZ_samples[i];
				gyroZ_samples[i] = gyroZ_samples[j];
				gyroZ_samples[j] = temp;
			}
		}
	}

	// **Step 3: Compute offset using median filter (more robust than mean)**
	gyroZ_offset = gyroZ_samples[NUM_CALIBRATION_SAMPLES / 2];  // Median value for better stability
}

float normalizeAngle(float angle) {
	angle = fmodf(angle, 360.0f);
	if (angle < 0) angle += 360.0f;
	return angle;
}


/////// IR Sensor Functions////////
uint32_t ADC_Read_Channel(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;  // dynamically set ADC channel
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
		return 0;  // error handling
	}
	uint32_t adcVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adcVal;
}

uint32_t read_IR_Left(void) {
	return ADC_Read_Channel(ADC_CHANNEL_11); // PC1
}

uint32_t read_IR_Right(void) {
	return ADC_Read_Channel(ADC_CHANNEL_12); // PC2
}

float IR_ADC_to_Distance(uint32_t adcValue) {
    if (adcValue > 3000) return 5.0f;        // ~5 cm (very close)
    else if (adcValue > 2500) return 7.0f;   // approximately 7 cm
    else if (adcValue > 2000) return 10.0f;
    else if (adcValue > 1500) return 15.0f;
    else if (adcValue > 1000) return 20.0f;
    else if (adcValue > 500) return 30.0f;

    return 50.0f;  // Default if far away or sensor out of range
}

/////// IR Sensor End////////


////////////////////ENCODER END////////////////////
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	OLED_Clear();
	int counter = 0;
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		sprintf((char*)OLEDBuffer, "Count: %d", counter);
		OLED_ShowString(10, 10, OLEDBuffer);
		//snprintf((char*)OLEDBuffer, sizeof(OLEDBuffer), "Yaw: %d", (int)heading);
		//OLED_ShowString(10, 30, OLEDBuffer);
		OLED_Refresh_Gram();
		counter++;
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_commandTask */
/**
 * @brief Function implementing the Command_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_commandTask */
void commandTask(void *argument)
{
	/* USER CODE BEGIN commandTask */
	uint32_t u32Cmd;
	uint16_t distanceCmd = 0, angleCmd = 0;

	/* Infinite loop */
	for(;;)
	{
		if (PopFromQueue_MTR_Cmd(&u32Cmd))
		{
			speedCmd = ((u32Cmd % 100000) / 1000.0f) * MAX_ENCODER_SPEED;
			commandStatus = true;
			cmdID = u32Cmd / 100000;
			if(cmdID != MOVE_FORWARD && cmdID != MOVE_BACKWARD && cmdID != STOP && cmdID != SNAP)
			{
				angleCmd      = u32Cmd % 1000;
				targetAngle = normalizeAngle(heading + angleCmd);
				targetDistance  = 0;
			}
			else
			{
				distanceCmd = (u32Cmd % 1000) * PULSE_PER_DISTANCE; // Convert duty cycle to encoder count
				targetDistance = distanceCmd;
				targetAngle = 0;
			}

			//snprintf(debugBuffer, sizeof(debugBuffer),
			//		"CMD: %lu | SpeedCmd: %u | DistanceCmd: %u\n",
			//		u32Cmd, speedCmd, targetDistance);
			//SendMsg_RPI((uint8_t*)debugBuffer, strlen(debugBuffer));

			//float newTarget = heading;

			// **Execute Command**
			switch (cmdID)
			{
			case MOVE_FORWARD:
				faceFront();
				leftTotalCount = 0; // Reset encoder counts
				rightTotalCount = 0;
				__HAL_TIM_SET_COUNTER(&htim2, 0);
				__HAL_TIM_SET_COUNTER(&htim4, 0);
				resetEncoderFlag = true;
				if (osMutexAcquire(headingMutex, osWaitForever) == osOK) {
					targetHeading = heading;
					osMutexRelease(headingMutex);
				}
				setMotorForward();
				//PID_init(&left_pid, PID_POSITION, pid_param_left, max_out, max_iout);
				//PID_init(&right_pid, PID_POSITION, pid_param_right, max_out, max_iout);
				commandStatus = true; // Set command status to true
				break;
			case MOVE_BACKWARD:
				faceFront();
				leftTotalCount = 0; // Reset encoder counts
				rightTotalCount = 0;
				__HAL_TIM_SET_COUNTER(&htim2, 0);
				__HAL_TIM_SET_COUNTER(&htim4, 0);
				resetEncoderFlag = true;
				osDelay(100);
				if (osMutexAcquire(headingMutex, osWaitForever) == osOK) {
					targetHeading = heading;
					osMutexRelease(headingMutex);
				}
				setMotorBackward();
				//PID_init(&left_pid, PID_POSITION, pid_param_left, max_out, max_iout);
				//PID_init(&right_pid, PID_POSITION, pid_param_right, max_out, max_iout);
				commandStatus = true; // Set command status to true
				break;
			case TURN_FORWARD_LEFT:
				targetAngle = normalizeAngle(heading - (u32Cmd % 1000));
				targetDistance = 0;
				turnLeft();
				osDelay(100);
				setMotorForward();
				Hdirection = 1;
				break;
			case TURN_FORWARD_RIGHT:
				targetAngle = normalizeAngle(heading + (u32Cmd % 1000));
				targetDistance = 0;
				turnRight();
				osDelay(100);
				setMotorForward();
				Hdirection = 2;
				break;
			case TURN_BACKWARD_LEFT:
				targetAngle = normalizeAngle(heading + (u32Cmd % 1000));
				targetDistance = 0;
				turnLeft();
				osDelay(100);
				setMotorBackward();
				Hdirection = 3;
				break;
			case TURN_BACKWARD_RIGHT:
				targetAngle = normalizeAngle(heading - (u32Cmd % 1000));
				targetDistance = 0;
				turnRight();
				osDelay(100);
				setMotorBackward();
				Hdirection = 4;
				break;
			case OFFSET_FORWARD_LEFT:
				targetAngle = normalizeAngle(heading - (u32Cmd % 1000));
				targetDistance = 0;
				offsetLeft();
				osDelay(100);
				setMotorForward();
				Hdirection = 1;
				break;
			case OFFSET_FORWARD_RIGHT:
				targetAngle = normalizeAngle(heading + (u32Cmd % 1000));
				targetDistance = 0;
				offsetRight();
				osDelay(100);
				setMotorForward();
				Hdirection = 2;
				break;
			case OFFSET_BACKWARD_LEFT:
				targetAngle = normalizeAngle(heading + (u32Cmd % 1000));
				targetDistance = 0;
				offsetLeft();
				osDelay(100);
				setMotorBackward();
				Hdirection  = 3;
				break;
			case OFFSET_BACKWARD_RIGHT:
				targetAngle = normalizeAngle(heading - (u32Cmd % 1000));
				targetDistance = 0;
				offsetRight();
				osDelay(100);
				setMotorBackward();
				Hdirection = 4;
				break;
			case STOP:
				motorStop();
				osDelay(100);
				SendMsg_RPI((uint8_t*)"ACK\n", strlen("ACK\n"));
				faceFront();
				commandStatus = false;
				targetDistance = 0;
				targetAngle = 0;
				break;
			case SNAP:
				SendMsg_RPI((uint8_t*)"SNAP\n", strlen("SNAP\n"));
				break;
			}
		}
		osDelay(100);
	}
	/* USER CODE END commandTask */
}

/* USER CODE BEGIN Header_imuTask */
/**
 * @brief Function implementing the IMU_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_imuTask */
void imuTask(void *argument)
{
	/* USER CODE BEGIN imuTask */
	// 1. Calibrate offset at startup (robot must be still)
	CalibrateGyroZOffset(&hi2c1);

	// 2. Initialize heading to 0
	heading = 0.0f;

	// Remember last time for delta-t
	uint32_t lastTick = HAL_GetTick();

	/* Infinite loop */
	for(;;)
	{
		// 3. Compute time step
		uint32_t currentTick = HAL_GetTick();
		float dt = (currentTick - lastTick) / 1000.0f;  // ms -> s
		lastTick = currentTick;

		// 4. Read raw gyroZ
		int16_t gyroZ_raw = 0;
		__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ_raw);

		// 5. Subtract offset
		int16_t gyroZ_corrected = gyroZ_raw - gyroZ_offset;

		// 6. Convert to deg/s
		float gyroZ_dps = gyroZ_corrected / GYRO_SENSITIVITY_SCALE_FACTOR_2000DPS;
		if (osMutexAcquire(headingMutex, osWaitForever) == osOK) {
			heading -= gyroZ_dps * dt;
			heading = fmodf(heading, 360.0f);
			if (heading < 0.0f) heading += 360.0f;
			osMutexRelease(headingMutex);  // 🔹 Unlock the mutex
		}

		// Optional: debug print
		//snprintf(debugBuffer, sizeof(debugBuffer), "Heading: %.2f\r\n", heading);
		//SendMsg_RPI((uint8_t*)debugBuffer, strlen(debugBuffer));
		osDelay(50);
	}

	/* USER CODE END imuTask */
}

/* USER CODE BEGIN Header_pidTask */
/**
 * @brief Function implementing the PID_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_pidTask */
void pidTask(void *argument)
{
	/* USER CODE BEGIN pidTask */
	uint16_t leftCount1 = 0, rightCount1 = 0,
			leftCount2 = 0, rightCount2 = 0;
	uint32_t leftDelta = 0, rightDelta = 0;

	float kp_heading = 4.2, ki_heading = 0.1, kd_heading = 1.2;

	static float distance_error_sum = 0.0f;
	static float distance_last_error = 0.0f;

	leftCount1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
	rightCount1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

	if (commandStatus && targetHeading == 0) {
		targetHeading = heading;
	}

	/* Infinite loop */
	for(;;)
	{
		if (commandStatus)
		{
			if (resetEncoderFlag)
			{
				leftCount1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
				rightCount1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
				resetEncoderFlag = false;
			}

			if(targetDistance != 0 && targetAngle == 0)
			{
				// Get encoder readings
				leftCount2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
				rightCount2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

				// Compute encoder delta
				leftDelta = getEncoderDelta(leftCount1, leftCount2, &htim2);
				rightDelta = getEncoderDelta(rightCount1, rightCount2, &htim4);

				// Accumulate total distance traveled
				leftTotalCount += leftDelta;
				rightTotalCount += rightDelta;

				// Compute motor output using PID
				if(cmdID == MOVE_FORWARD){
					leftOutputPWM = PID_calc(&forward_pid, leftTotalCount, targetDistance);
					leftOutputPWM*= 0.6;
				}
				else {
					leftOutputPWM = PID_calc(&backward_pid, leftTotalCount, targetDistance);
				}
				rightOutputPWM = leftOutputPWM;
				//original
				//leftOutputPWM = PID_calc(&left_pid, leftTotalCount, targetDistance);
				//rightOutputPWM = PID_calc(&right_pid, rightTotalCount, targetDistance);

				// Get IMU heading data
				float currentHeading;
				if (osMutexAcquire(headingMutex, osWaitForever) == osOK) {
					currentHeading = heading;
					osMutexRelease(headingMutex);
				}
				float headingError = targetHeading - currentHeading;

				// Normalize heading error (-180 to 180)
				if (headingError > 180.0f) headingError -= 360.0f;
				else if (headingError < -180.0f) headingError += 360.0f;

				float heading_derivative = headingError - distance_last_error;
				distance_error_sum += headingError;

				// Integral Windup Prevention
				if (distance_error_sum > 50.0f) distance_error_sum = 50.0f;
				if (distance_error_sum < -50.0f) distance_error_sum = -50.0f;

				// Compute Servo Angle Correction
				float servoCorrection;
				if (cmdID == MOVE_FORWARD) {
					servoCorrection = CENTER_POS_PWM + (kp_heading * headingError) +
							(ki_heading * distance_error_sum) +
							(kd_heading * heading_derivative);
				} else {
					servoCorrection = CENTER_POS_PWM - (kp_heading * headingError) -
							(ki_heading * distance_error_sum) -
							(kd_heading * heading_derivative);
				}


				// Clamp servo position within allowed range
				if (servoCorrection > (CENTER_POS_PWM + RIGHT_DELTA))
					servoCorrection = CENTER_POS_PWM + RIGHT_DELTA;
				if (servoCorrection < (CENTER_POS_PWM - LEFT_DELTA))
					servoCorrection = CENTER_POS_PWM - LEFT_DELTA;

				// Set Servo Position for Steering
				htim1.Instance->CCR4 = (uint16_t)servoCorrection;

				// Debug Heading & Servo Correction
				//snprintf(debugBuffer, sizeof(debugBuffer), "Heading: %.2f, Target: %.2f, Error: %.2f, Servo: %.2f\n",
				//		currentHeading, targetHeading, headingError, servoCorrection);
				//SendMsg_RPI((uint8_t*)debugBuffer, strlen(debugBuffer));

				distance_last_error = headingError;

				// Ensure PWM Values Stay in Range
				if (leftOutputPWM < 500) leftOutputPWM = 500;
				if (rightOutputPWM < 500) rightOutputPWM = 500;
				if (leftOutputPWM > MAX_MOTOR_PWM) leftOutputPWM = MAX_MOTOR_PWM;
				if (rightOutputPWM > MAX_MOTOR_PWM) rightOutputPWM = MAX_MOTOR_PWM;

				// Set motor PWM
				setLeftPWM(leftOutputPWM * 0.85);
				setRightPWM(rightOutputPWM * 0.85);



				// **Stop Condition: When distance is reached**
				if (leftTotalCount >= targetDistance || rightTotalCount >= targetDistance)
				{
					motorStop();
					commandStatus = false;
					leftTotalCount = 0;
					rightTotalCount = 0;
					targetDistance = 0;
					htim1.Instance->CCR4 = CENTER_POS_PWM;
					SendMsg_RPI((uint8_t*)"ACK\n", strlen("ACK\n"));
				}
			}
			// Update last encoder readings
			leftCount1 = leftCount2;
			rightCount1 = rightCount2;
		}

		osDelay(20);
	}
	/* USER CODE END pidTask */
}

/* USER CODE BEGIN Header_Ultrasonic_Task */
/**
 * @brief Function implementing the UltrasonicTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Ultrasonic_Task */
void Ultrasonic_Task(void *argument)
{
	/* USER CODE BEGIN Ultrasonic_Task */
	uint32_t distance_cm = 0U;
	/* Infinite loop */
	for(;;)
	{
		TriggerPulse();  // Send trigger pulse
		uint32_t echoTime = MeasureEchoTime();  // Measure pulse duration
		distance_cm = CalculateDistance(echoTime);  // Convert to cm

		// Debugging: Send the distance to RPI or display on OLED
		//		snprintf(debugBuffer, sizeof(debugBuffer), "Distance: %lu cm\n", (unsigned long)distance_cm);
		//		SendMsg_RPI((uint8_t*)debugBuffer, strlen(debugBuffer));

		// Update OLED Display
		//sprintf((char *)OLEDBuffer, "Dist: %lu cm", (unsigned long)distance_cm);
		//OLED_ShowString(10, 30, OLEDBuffer);
		//OLED_Refresh_Gram();
		osDelay(200);
	}
	/* USER CODE END Ultrasonic_Task */
}

/* USER CODE BEGIN Header_Heading_PID */
/**
 * @brief Function implementing the HeadingPID thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Heading_PID */
void Heading_PID(void *argument)
{
	/* USER CODE BEGIN Heading_PID */
	float kp_left = 0.75, ki_left = 0.002, kd_left = 0.6;  // Fine-tuned for left turns
	float kp_right = 2.0, ki_right = 0.2, kd_right = 0.32;  // Fine-tuned for right turns

	float kp_backward_left = 0.5, ki_backward_left = 0.001, kd_backward_left = 0.45;
	float kp_backward_right = 0.6, ki_backward_right = 0.0, kd_backward_right = 0.5;

	static float heading_error_sum = 0.0f;
	static float heading_last_error = 0.0f;

	/* Infinite loop */
	for(;;)
	{
		if(targetDistance == 0 && targetAngle != 0)
		{
			float currentHeading;
			if (osMutexAcquire(headingMutex, osWaitForever) == osOK) {
				currentHeading = heading;
				osMutexRelease(headingMutex);  // 🔹 Unlock mutex
			}
			float angleError = targetAngle - currentHeading;

			// If you only need to rotate e.g. +90 or -90, you may want to normalize angleError
			// so that it does the "shortest turn":
			if (angleError > 180.0f) {
				angleError -= 360.0f;
			} else if (angleError < -180.0f) {
				angleError += 360.0f;
			}

			float kp, ki, kd;
			if (Hdirection == 1) {  // Left turn
				kp = kp_left;
				ki = ki_left;
				kd = kd_left;
			} else if (Hdirection == 2) {  // Right turn
				kp = kp_right;
				ki = ki_right;
				kd = kd_right;
			} else if (Hdirection == 3) {  // Right turn
				kp = kp_backward_left;
				ki = ki_backward_left;
				kd = kd_backward_left;
			} else if (Hdirection == 4) {  // Right turn
				kp = kp_backward_right;
				ki = ki_backward_right;
				kd = kd_backward_right;
			}

			// Then do your PID control with angleError
			heading_error_sum += angleError;
			float heading_derivative = angleError - heading_last_error;

			// Calculate motor speed
			float motorSpeed = kp * angleError + ki * heading_error_sum + kd * heading_derivative;
			uint16_t dutyCycle = (uint16_t) (fabs(motorSpeed) * MAX_MOTOR_PWM);
			if (dutyCycle > MAX_MOTOR_PWM) dutyCycle = MAX_MOTOR_PWM;

			// Reduce speed when close to target
			if (fabs(angleError) < 15.0f) {
				dutyCycle *= 0.5;
			}

			if(Hdirection == 1){
				setLeftPWM(1600);
				setRightPWM(dutyCycle*0.7);
			} else if(Hdirection == 2){
				setRightPWM(700);
				setLeftPWM(dutyCycle);
			} else if (Hdirection == 3) {  // Backward Left turn
				setLeftPWM(2000);
				setRightPWM(dutyCycle*0.7);  // Slightly higher for smooth turning
			} else if (Hdirection == 4) {  // Backward Right turn
				setRightPWM(700);
				setLeftPWM(dutyCycle);  // Reduce slightly for better balance
			}
			//setLeftPWM(dutyCycle);
			//setRightPWM(dutyCycle);

			if (fabs(angleError) < 8.0f) {
				faceFront();
				motorStop();
				commandStatus = false;
				targetAngle = 0;
				Hdirection = 0;
				SendMsg_RPI((uint8_t*)"ACK\n", strlen("ACK\n"));
			}
			heading_last_error = angleError;
		}
		osDelay(100);
	}
	/* USER CODE END Heading_PID */
}

/* USER CODE BEGIN Header_IRSensor_Task */
/**
 * @brief Function implementing the IRSensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IRSensor_Task */
void IRSensor_Task(void *argument)
{
	/* USER CODE BEGIN IRSensor_Task */
	uint32_t irLeftVal, irRightVal;
	float obsDistLeft, obsDistRight;
	char debugBuffer[64];
	/* Infinite loop */
	for(;;)
	{
		irLeftVal = read_IR_Left();
		irRightVal = read_IR_Right();

		// Convert ADC values to distances (example calibration, modify as needed)
		obsDistLeft = IR_ADC_to_Distance(irLeftVal);
		obsDistRight = IR_ADC_to_Distance(irRightVal);

		// Send to RPI for debugging
		//snprintf(debugBuffer, sizeof(debugBuffer), "IR L:%.1fcm R:%.1fcm\n", obsDistLeft, obsDistRight);
		//SendMsg_RPI((uint8_t*)debugBuffer, strlen(debugBuffer));

		// If obstacle detected close enough, stop the robot
		if (obsDistLeft < 10.0f || obsDistRight < 10.0f) {  // obstacle within 10 cm
			motorStop();
			SendMsg_RPI((uint8_t*)"Obstacle Detected\n", strlen("Obstacle Detected\n"));
		}

		osDelay(100);
	}
	/* USER CODE END IRSensor_Task */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
