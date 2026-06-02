/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "string.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include "oled.h"
#include "motor.h"
#include "transmit.h"
#include "pid.h"
#include "pca9685.h"
#include "my_MPU6050.h"
#include "new-controls.h"
#include "lift.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_A_Pin GPIO_PIN_6
#define IN1_A_GPIO_Port GPIOE
#define GND_Pin GPIO_PIN_3
#define GND_GPIO_Port GPIOF
#define GNDF4_Pin GPIO_PIN_4
#define GNDF4_GPIO_Port GPIOF
#define IN4_B_Pin GPIO_PIN_7
#define IN4_B_GPIO_Port GPIOF
#define IN4_A_Pin GPIO_PIN_8
#define IN4_A_GPIO_Port GPIOF
#define GNDF9_Pin GPIO_PIN_9
#define GNDF9_GPIO_Port GPIOF
#define VCC_Pin GPIO_PIN_10
#define VCC_GPIO_Port GPIOF
#define GNDC0_Pin GPIO_PIN_0
#define GNDC0_GPIO_Port GPIOC
#define VCCC1_Pin GPIO_PIN_1
#define VCCC1_GPIO_Port GPIOC
#define VCCC2_Pin GPIO_PIN_2
#define VCCC2_GPIO_Port GPIOC
#define GNDC3_Pin GPIO_PIN_3
#define GNDC3_GPIO_Port GPIOC
#define GNDA2_Pin GPIO_PIN_2
#define GNDA2_GPIO_Port GPIOA
#define IN2_A_Pin GPIO_PIN_3
#define IN2_A_GPIO_Port GPIOA
#define IN2_B_Pin GPIO_PIN_4
#define IN2_B_GPIO_Port GPIOA
#define GNDB2_Pin GPIO_PIN_2
#define GNDB2_GPIO_Port GPIOB
#define VCCF11_Pin GPIO_PIN_11
#define VCCF11_GPIO_Port GPIOF
#define VCCF14_Pin GPIO_PIN_14
#define VCCF14_GPIO_Port GPIOF
#define GNDE7_Pin GPIO_PIN_7
#define GNDE7_GPIO_Port GPIOE
#define GNDE8_Pin GPIO_PIN_8
#define GNDE8_GPIO_Port GPIOE
#define DIR_Pin GPIO_PIN_10
#define DIR_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_11
#define EN_GPIO_Port GPIOE
#define key_Pin GPIO_PIN_12
#define key_GPIO_Port GPIOE
#define I2C2_V_Pin GPIO_PIN_15
#define I2C2_V_GPIO_Port GPIOE
#define GNDD8_Pin GPIO_PIN_8
#define GNDD8_GPIO_Port GPIOD
#define VCCD9_Pin GPIO_PIN_9
#define VCCD9_GPIO_Port GPIOD
#define IN3_B_Pin GPIO_PIN_10
#define IN3_B_GPIO_Port GPIOD
#define IN3_A_Pin GPIO_PIN_11
#define IN3_A_GPIO_Port GPIOD
#define GNDG2_Pin GPIO_PIN_2
#define GNDG2_GPIO_Port GPIOG
#define VCCG3_Pin GPIO_PIN_3
#define VCCG3_GPIO_Port GPIOG
#define I2C2_G_Pin GPIO_PIN_4
#define I2C2_G_GPIO_Port GPIOG
#define GNDG5_Pin GPIO_PIN_5
#define GNDG5_GPIO_Port GPIOG
#define GNDG6_Pin GPIO_PIN_6
#define GNDG6_GPIO_Port GPIOG
#define VCCG7_Pin GPIO_PIN_7
#define VCCG7_GPIO_Port GPIOG
#define VCCG8_Pin GPIO_PIN_8
#define VCCG8_GPIO_Port GPIOG
#define USART1_G_Pin GPIO_PIN_8
#define USART1_G_GPIO_Port GPIOA
#define V2_Pin GPIO_PIN_11
#define V2_GPIO_Port GPIOA
#define G2_Pin GPIO_PIN_12
#define G2_GPIO_Port GPIOA
#define USART1_V_Pin GPIO_PIN_10
#define USART1_V_GPIO_Port GPIOC
#define USART2_G_Pin GPIO_PIN_7
#define USART2_G_GPIO_Port GPIOD
#define GNDG11_Pin GPIO_PIN_11
#define GNDG11_GPIO_Port GPIOG
#define VCCG15_Pin GPIO_PIN_15
#define VCCG15_GPIO_Port GPIOG
#define IN1_B_Pin GPIO_PIN_4
#define IN1_B_GPIO_Port GPIOB
#define VCCB5_Pin GPIO_PIN_5
#define VCCB5_GPIO_Port GPIOB
#define GNDB8_Pin GPIO_PIN_8
#define GNDB8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern volatile uint32_t pulse_count;   // 已发送脉冲计 ????
extern uint32_t target_pulses;       // 目标脉冲数（1600 ????/转）
extern uint16_t pulse_freq;
extern uint8_t is_moving;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
