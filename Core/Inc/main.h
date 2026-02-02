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
void delay_us(uint16_t time);
void LED_Control(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin , uint16_t Staus);
void Set_PWM(int G_num , int pwm_val);
void Read_Reapeter_Data(uint8_t Com_Status);
void Read_Reapeter_Single_Data(uint8_t Address, uint8_t Com_Status);
void Set_Relay(uint8_t num,uint8_t Port ,uint8_t in_status, uint8_t out_status );
void Check_Relay(void);


void Set_BAUDRATE(int BAUDRATE, int Port);
void UART_ReInit(int BAUDRATE);

void Read_Reapeter_Loop_Data(int Loop);
void PLC_Enable(int Loop, int Enable);

void PLC_Enable(int Loop, int Enable);
void Check_Over_Current(void);

void Relay_Control(uint16_t Port, uint16_t Staus, uint16_t Wate_Time, uint16_t Latch_Num);

void Read_Num_Reapeter_Data(uint8_t Com_Status,uint8_t Read_Num);

void Read_Test_Info_Data(void);

void ReSet_UART1_IO(void);
void Set_UART1_IO(void);

void Read_Repeater_Version_All(uint8_t Com_Status);
void Read_Sub_Version(void);

void Read_Reapeter_Set_Data(uint8_t Com_Status, uint8_t Address);

void Analog_test_read(void);

uint8_t Read_Repeater_Version(int num, uint8_t Com_Status);

uint8_t Check_Line_Status(int num, uint8_t Com_Status);
uint8_t Check_Line_Data(int check_val);

extern uint8_t Rep_ISO_In_Open;
extern uint8_t Rep_ISO_In_Short;
extern uint8_t Rep_ISO_Out_Open;
extern uint8_t Rep_ISO_Out_Short;
extern uint8_t In_Volt_St, Out_Vol_St;
extern uint8_t In_Volt_Val, Out_Vol_Val;
extern uint8_t Rep_In_Relay_Mode;
extern uint8_t Rep_Out_Relay_Mode;

extern uint8_t Loop_Back_Start;
extern uint8_t Loop_Back_First;

extern uint8_t End_REPEATER_Number;

extern uint8_t In_Relay_Mode;
extern uint8_t Out_Relay_Mode;

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

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;


#define OUT_PORT  	0
#define IN_PORT 	1

#define Realay_On  1
#define Realay_Off 0

#define LED_On  1
#define LED_Off 0

#define Loop_Back_Enable  1
#define Loop_Back_Disable 0

extern uint8_t CCU_Address ;

#define Repeater_Header_Number  6
#define Repeater_Number         220
#define Repeater_IN_OUT         2
//#define Repeater_Data_Number    6
#define Repeater_Data_Number    5
#define Repeater_Crc_Number     2
#define Repeater_End_Number     2
//#define Repeater_Data_Total     1260
#define Repeater_Data_Total     1110

#define Group_Register_Data_Number  39

#define Group_Charge_Register_Number  508

#define Group_Charge_Time_Number  9
#define Repeater_Set_Data_Len     14

#define Max_Wate_ms  15

#define Max_Wate_udelay	500
//#define Max_Wate_Recever	35*(1000 / Max_Wate_udelay) // under 100, Set to multiples of 8
#define Max_Wate_Recever	35*(1000 / Max_Wate_udelay) // under 100, Set to multiples of 8
#define Pow_Bod_Max_Number 8

#define Pow_Bod_Header_Number  6
#define Pow_Bod_Data_Number   6
#define Pow_Bod_CkSum_Number  1
#define Pow_Bod_End_Number     2
#define Pow_Bod_Data_Total  15


//Array to store lower 8 relay panel power board information
//Pow_Bod_Data[8][16]
extern uint8_t Pow_Bod_Data[Pow_Bod_Max_Number][Pow_Bod_Header_Number + Pow_Bod_Data_Number + Pow_Bod_CkSum_Number + Pow_Bod_End_Number];
//Rep_Pow_Bod_Data[16]
extern uint8_t Rep_Pow_Bod_Data[Pow_Bod_Header_Number + Pow_Bod_Data_Number + Pow_Bod_CkSum_Number + Pow_Bod_End_Number];

//Array to store sub-64 system information
// Group_Data[1260]
extern uint8_t Group_Data[Repeater_Header_Number +(Repeater_Number * Repeater_Data_Number) + Repeater_Crc_Number + Repeater_End_Number ];

//Array that stores system information to be reported to the parent
//Group_Rep_Group_Data[1260]
extern uint8_t Rep_Group_Data[Repeater_Header_Number +(Repeater_Number * Repeater_Data_Number) + Repeater_Crc_Number + Repeater_End_Number];

//Array to store sub-250 system registration information
extern uint8_t REPEATER_Regster[Repeater_Number];
extern uint8_t REPEATER_Regster_Num;

//Array to Possible Communicatin In Out Out : 0 , In : 1
extern uint8_t REPEATER_Possible[Repeater_Number];

extern uint8_t REPEATER_Out_Loop_End;

//Array to store the charge information of the lower 250 systems
extern uint8_t REPEATER_Charge_Regster[4];
extern uint8_t REPEATER_Acc_Com_Data[Repeater_Number];


//Array Repeater accumulation setting
#define REPEATER_Acc_Set_Num 229
extern uint8_t REPEATER_Acc_Set_Data[Repeater_Number];

//Array to store accumulation time information for lower 64 systems, does not operate if time is 0
//Group_Charge_Time[9]
extern uint8_t REPEATER_Charge_Time[Group_Charge_Time_Number];

//Array to store sub-250 system Response information
extern uint8_t REPEATER_Response_Regster[Repeater_IN_OUT][Repeater_Number];


//Define UI uart receiving array
#define UI_UART_buf_len 2048
extern uint8_t UI_UART_TX_buf[UI_UART_buf_len] ;
extern uint8_t UI_UART_RX_buf[UI_UART_buf_len];
extern uint8_t UI_UART_Tx_buf_tmp[UI_UART_buf_len];
extern uint8_t UI_UART_RX_buf_tmp[UI_UART_buf_len];
extern uint8_t UI_UART_State ;					// Define reception start state
extern int UI_UART_buf_count;					// Save the length of received data after starting reception
extern int UI_UART_buf_count_tmp;				// Variable to store the length of the received data before initializing it when reception is completed
extern uint8_t UI_UART_Receive_complete;		// Variable indicating that reception has been completed
extern int UI_UART_buf_count_Save;

#define UART_buf_len 32
extern uint8_t UART_TX_buf[2][UART_buf_len] ;
extern uint8_t UART_RX_buf[2][UART_buf_len] ;
extern uint8_t UART_RX_buf_tmp[2][UART_buf_len] ;
extern uint8_t UART_RX_buf_temp;
extern uint8_t UART_State[2];					// Define reception start state
extern int UART_buf_count[2];					// Save the length of received data after starting reception
extern int UART_buf_count_tmp[2];				// Variable to store the length of the received data before initializing it when reception is completed
extern uint8_t UART_Receive_complete[2];			// Variable indicating that reception has been completed
extern uint8_t UART_Transmit_complete[2];			// Variable indicating that reception has been completed
extern uint8_t UART_DMA_CNT[2];					// Time check variable after DMA start
extern int UART_DMA_ERR_CNT[2];				//DMA error count

#define Loop_Normal		0
#define Loop_Start_Err	1
#define Loop_Err		2
#define Loop_Short		3
#define Loop_Open		4

#define Com_On	0
#define Com_Off	1

#define PLC_On	0
#define PLC_Off	1

extern float Loop_In_Voltage;
extern float Loop_Out_Voltage;

extern uint8_t Loop_In_Status;
extern uint8_t Loop_Out_Status;

extern uint8_t Com_Out_Status;
extern uint8_t Com_In_Status;

extern int UART_BAUDRATE;

extern uint8_t UART2_TX_DMA_complete,UART4_TX_DMA_complete;

#define Read_Sub_Cnt 3
extern uint8_t Reset_UI_uart, Reset_uart;
extern uint8_t REPEATER_Read_Regster[Repeater_Number];

extern uint8_t REPEATER_first_fire;
#define In_Short_Cnt 5
extern uint8_t REPEATER_In_Short[Repeater_Number][In_Short_Cnt];

#define Version_Cnt 6
extern uint8_t REPEATER_Version[Repeater_Number+1][Version_Cnt];
#define Rep_Version_Data_Number 1557
extern uint8_t Rep_Version_Data[Rep_Version_Data_Number];

extern uint8_t Info_Data_Version[6];

extern int CCU_Mode;

#define Latch_mode	1
#define Relay_mode	0
extern uint8_t Boot_Mode;

extern uint8_t Loop_mode;
#define Loop_Set		1
#define Normal_Set		0

extern uint8_t Sub_Version_Read_Cnt;
////////////////////////////
extern uint8_t Link_Table_Year_Month;
extern uint8_t Link_Table__Day;
extern uint8_t Link_Table_Hour;
extern uint8_t Serial_Num[7];

extern uint8_t Sub_Set_Mode , Mini_Mode;

extern uint8_t Rep_Set_Mode;
#define Rep_Number_30 	0
#define Rep_Number_220 	1

#define Latch_Hold_Time 	10
#define Latch_Change_Num 	5


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
