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
#include "jlinkp.h"
#include "stdint.h"
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
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_2
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_3
#define PWM4_GPIO_Port GPIOA
#define LED_PWM1_Pin GPIO_PIN_6
#define LED_PWM1_GPIO_Port GPIOA
#define LED_PWM3_Pin GPIO_PIN_7
#define LED_PWM3_GPIO_Port GPIOA
#define LED_PWM4_Pin GPIO_PIN_0
#define LED_PWM4_GPIO_Port GPIOB
#define LED_PWM2_Pin GPIO_PIN_1
#define LED_PWM2_GPIO_Port GPIOB
#define NRF24L01_CSN_Pin GPIO_PIN_12
#define NRF24L01_CSN_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define NRF24L01_IRQ_Pin GPIO_PIN_8
#define NRF24L01_IRQ_GPIO_Port GPIOA
#define NRF24L01_CE_Pin GPIO_PIN_8
#define NRF24L01_CE_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
