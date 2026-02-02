/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32u0xx_hal.h"

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
#define UART1_D1_Pin GPIO_PIN_13
#define UART1_D1_GPIO_Port GPIOC
#define UART1_D2_Pin GPIO_PIN_14
#define UART1_D2_GPIO_Port GPIOC
#define LOOP_OUT_OFF_SET_Pin GPIO_PIN_15
#define LOOP_OUT_OFF_SET_GPIO_Port GPIOC
#define MCU_PLC1_EN_Pin GPIO_PIN_0
#define MCU_PLC1_EN_GPIO_Port GPIOF
#define UART1_TXD_EN_Pin GPIO_PIN_1
#define UART1_TXD_EN_GPIO_Port GPIOF
#define UART2_D1_Pin GPIO_PIN_0
#define UART2_D1_GPIO_Port GPIOC
#define LOOP_OUT_ON_SET_Pin GPIO_PIN_1
#define LOOP_OUT_ON_SET_GPIO_Port GPIOC
#define COMM1_FS_Pin GPIO_PIN_2
#define COMM1_FS_GPIO_Port GPIOC
#define UART2_D2_Pin GPIO_PIN_3
#define UART2_D2_GPIO_Port GPIOC
#define MCU_PLC2_EN_Pin GPIO_PIN_4
#define MCU_PLC2_EN_GPIO_Port GPIOA
#define LOOP_IN_OFF_SET_Pin GPIO_PIN_7
#define LOOP_IN_OFF_SET_GPIO_Port GPIOA
#define COMM2_FS_Pin GPIO_PIN_4
#define COMM2_FS_GPIO_Port GPIOC
#define LOOP_IN_ON_SET_Pin GPIO_PIN_5
#define LOOP_IN_ON_SET_GPIO_Port GPIOC
#define UART2_TXD_EN_Pin GPIO_PIN_0
#define UART2_TXD_EN_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_1
#define RS485_DE_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_2
#define RS485_RE_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_12
#define SPI_CS_GPIO_Port GPIOB
#define DIP0_Pin GPIO_PIN_6
#define DIP0_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_7
#define DIP1_GPIO_Port GPIOC
#define DIP2_Pin GPIO_PIN_8
#define DIP2_GPIO_Port GPIOC
#define DIP3_Pin GPIO_PIN_9
#define DIP3_GPIO_Port GPIOC
#define DIP4_Pin GPIO_PIN_8
#define DIP4_GPIO_Port GPIOA
#define DIP5_Pin GPIO_PIN_9
#define DIP5_GPIO_Port GPIOA
#define DIP6_Pin GPIO_PIN_10
#define DIP6_GPIO_Port GPIOA
#define DIP7_Pin GPIO_PIN_15
#define DIP7_GPIO_Port GPIOA
#define ERR_UART2_LED_Pin GPIO_PIN_10
#define ERR_UART2_LED_GPIO_Port GPIOC
#define ERR_UART1_LED_Pin GPIO_PIN_11
#define ERR_UART1_LED_GPIO_Port GPIOC
#define UART2_RXD_LED_Pin GPIO_PIN_12
#define UART2_RXD_LED_GPIO_Port GPIOC
#define UART1_RXD_LED_Pin GPIO_PIN_2
#define UART1_RXD_LED_GPIO_Port GPIOD
#define UART2_TXD_LED_Pin GPIO_PIN_4
#define UART2_TXD_LED_GPIO_Port GPIOB
#define UART1_TXD_LED_Pin GPIO_PIN_5
#define UART1_TXD_LED_GPIO_Port GPIOB
#define BOOT_MODE_Pin GPIO_PIN_3
#define BOOT_MODE_GPIO_Port GPIOF
#define RUN_LED2_Pin GPIO_PIN_8
#define RUN_LED2_GPIO_Port GPIOB
#define RUN_LED1_Pin GPIO_PIN_9
#define RUN_LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
