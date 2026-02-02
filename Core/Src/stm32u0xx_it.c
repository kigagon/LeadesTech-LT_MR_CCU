/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32u0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32u0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart4_tx;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SVC instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32U0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles DMAMUX_OVR_IT + DMA1 channel 4 to 7 + DMA2 channel 1 to 5.
  */
void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn 0 */

  /* USER CODE END DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  HAL_DMA_IRQHandler(&hdma_usart4_tx);
  /* USER CODE BEGIN DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn 1 */

  /* USER CODE END DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn 1 */
}

/**
  * @brief This function handles TIM1 Capture Compare Interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM6, DAC and LPTIM1 global Interrupts (combined with EXTI 31).
  */
void TIM6_DAC_LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 0 */

  /* USER CODE END TIM6_DAC_LPTIM1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 1 */

  /* USER CODE END TIM6_DAC_LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles TIM7 and LPTIM2 global Interrupt (combined with EXTI 32).
  */
void TIM7_LPTIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_LPTIM2_IRQn 0 */

  /* USER CODE END TIM7_LPTIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_LPTIM2_IRQn 1 */


  /* USER CODE END TIM7_LPTIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM15 + LPTIM3 global interrupt (combined with EXTI 33).
  */
void TIM15_LPTIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_LPTIM3_IRQn 0 */

  /* USER CODE END TIM15_LPTIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM15_LPTIM3_IRQn 1 */

  /* USER CODE END TIM15_LPTIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt (combined with EXTI 26) + LPUART2 global interrupt (combined with EXTI lines 35).
  */
void USART2_LPUART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_LPUART2_IRQn 0 */

  /* USER CODE END USART2_LPUART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_LPUART2_IRQn 1 */

  UART_RX_buf_tmp[1][UART_buf_count[1]]= USART2->RDR;
  UART_buf_count[1]++;

  if((UART_RX_buf_tmp[1][UART_buf_count[1] -1 ] == 0x03) &(UART_RX_buf_tmp[1][UART_buf_count[1] -1 - 6 ] == 0x02) ) {
    for(int k=0;k<7;k++){
    	UART_RX_buf[1][k] = UART_RX_buf_tmp[1][UART_buf_count[1] - 7 + k];
    }
    UART_State[1] = 0;
    UART_buf_count[1] = 0;
    UART_Receive_complete[1] = 1;
    for(int k=0;k<UART_buf_len;k++){
    	UART_RX_buf_tmp[1][k] = 0;
    }
  }

  if(UART_buf_count[1] >= UART_buf_len){
	UART_State[1] = 0;
	UART_buf_count[1] = 0;
	UART_Receive_complete[1] = 0;
    for(int k=0;k<UART_buf_len;k++){
    	UART_RX_buf_tmp[1][k] = 0;
    }
  }


  /* USER CODE END USART2_LPUART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 (combined with EXTI 24) + LPUART1 global interrupt (combined with EXTI lines 28).
  */
void USART3_LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_LPUART1_IRQn 0 */

  /* USER CODE END USART3_LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN USART3_LPUART1_IRQn 1 */


	UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp] = LPUART1->RDR;
	UI_UART_buf_count_tmp++;

	if(UI_UART_buf_count_tmp > 2){
	  if((UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp-3] == 0x53) &&(UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp-2] == 0x54)
			  &&(UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp-1] == 0x55))
	  {
		  UI_UART_State = 1;
		  UI_UART_buf_count = 2;
		  UI_UART_RX_buf[0] = UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp-3];
		  UI_UART_RX_buf[1] = UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp-2];

	  }
	}

	  if(UI_UART_State == 1){
		  UI_UART_RX_buf[UI_UART_buf_count] = UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp-1];
		  UI_UART_buf_count++;

		  if((UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp -2 ] == 0x45) &&(UI_UART_RX_buf_tmp[UI_UART_buf_count_tmp - 1] == 0x44)) {
			  UI_UART_Receive_complete = 1;
			  UI_UART_State = 0;
			  UI_UART_buf_count_tmp = 0;
			  UI_UART_buf_count_Save = UI_UART_buf_count;
			  UI_UART_buf_count = 0;

		  }
	  }

	  if(UI_UART_buf_count > UI_UART_buf_len -1){
		  UI_UART_State = 0;
		  UI_UART_buf_count = 0;
		  UI_UART_buf_count_tmp = 0;
		  UI_UART_Receive_complete = 0;
		  for(int k=0;k<UI_UART_buf_len;k++){
			UI_UART_RX_buf_tmp[k] = 0;
		  }
		}
  /* USER CODE END USART3_LPUART1_IRQn 1 */
}

/**
  * @brief This function handles USART4  global interrupt (combined with EXTI 20) + LPUART3 (combined with EXTI lines 34).
  */
void USART4_LPUART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART4_LPUART3_IRQn 0 */

  /* USER CODE END USART4_LPUART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN USART4_LPUART3_IRQn 1 */



	UART_RX_buf_tmp[0][UART_buf_count[0]]= USART4->RDR;
    UART_buf_count[0]++;

    if((UART_RX_buf_tmp[0][UART_buf_count[0] -1 ] == 0x03) &(UART_RX_buf_tmp[0][UART_buf_count[0] -1 - 6 ] == 0x02) ) {
      for(int k=0;k<7;k++){
      	UART_RX_buf[0][k] = UART_RX_buf_tmp[0][UART_buf_count[0] - 7 + k];
      }
      UART_State[0] = 0;
      UART_buf_count[0] = 0;
      UART_Receive_complete[0] = 1;
      for(int k=0;k<UART_buf_len;k++){
      	UART_RX_buf_tmp[0][k] = 0;
      }
    }

    if(UART_buf_count[0] >= UART_buf_len){
  	UART_State[0] = 0;
  	UART_buf_count[0] = 0;
  	UART_Receive_complete[0] = 0;
      for(int k=0;k<UART_buf_len;k++){
      	UART_RX_buf_tmp[0][k] = 0;
      }
    }

  /* USER CODE END USART4_LPUART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
