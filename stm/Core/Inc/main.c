/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define CIN2_Pin GPIO_PIN_5
#define CIN2_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define CIN1_Pin GPIO_PIN_12
#define CIN1_GPIO_Port GPIOE
#define ECHO_PIN_Pin GPIO_PIN_13
#define ECHO_PIN_GPIO_Port GPIOD
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMC_Pin GPIO_PIN_8
#define PWMC_GPIO_Port GPIOC
#define HCSR04_TRIG_Pin GPIO_PIN_4
#define HCSR04_TRIG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define __Gyro_Read_Z(_I2C, readGyroData, gyroZ) ({ \
	HAL_I2C_Mem_Read(_I2C,ICM20948__I2C_SLAVE_ADDRESS_1 << 1, ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, readGyroData, 2, 0xFFFF); \
	gyroZ = readGyroData[0] << 8 | readGyroData[1]; \
})

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
