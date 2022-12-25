/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define CHARGE "charge"
#define DISCHARGE "discharge"
#define IDLE "idle"

typedef struct {
	float voltage;
	float current;
	double temperature;
} stBattery_t;

typedef struct {
	char buffer[10];
	bool update;
} stUSART_t;

typedef enum {
	RelayStatus_Idle = 0, RelayStatus_Charge, RelayStatus_Discharge
} RelayStatus_e;

typedef struct {
	uint16_t tick;
	bool update;
} stTimer_t;

typedef enum {
	batteryState_Idle = 0, batteryState_Charged, batteryState_Discharged
} batteryState_e;

typedef struct {
	float chargeCutOffVoltage;
	float dischargeCutOffVoltage;
} stTestParameter_t;

void Error_Handler(void);

#define relay1_Pin GPIO_PIN_4
#define relay1_GPIO_Port GPIOA
#define relay2_Pin GPIO_PIN_5
#define relay2_GPIO_Port GPIOA


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
