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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


// dip switch
// Dip0 . Dip1, Dip2 : add 1~8  , 9 ~12 : mini ,well
// Dip4 : 1: test mode ( 1~ 30  개만 동작) , 0: 기본 모드 ( 1~ 220  개 동작)
// Dip5 : 1 -> Loop_mode = Loop_Set , 0 -> Loop_mode = Normal_Set
// Dip6, Dip7 : 통신 속도 UART_BAUDRATE = 9600*0.5*(Read_Baud_val+2);

#include <stdio.h>

#include "UART_Funtions.h"
#include "Sub_Com.h"

#include "Analog_Process.h"
#include "ReadWriteFlash.h"

//Page : 256 byte , 16384
//Sector : 4096byte , 1024 (16 page)
//Block: 65536byte, 64 (16 sector , 256page)

// 256(250):input X  256(250):Output X 1byte = 65536byte = 1Block , block add : 0
// 32 group : 32 block
//Ccu : 126kb , 3block, block add : 33
//Com : 164kb , 4block, block add : 37


// 260106 W25Q 동작 확인 안됨
// 내부 플래쉬 사용
// 0 ~ 95Kbyte : 1 ~ 47 Page
// 96 ~ 127 Kbyte : 48 ~ 63 Page
// 48 Page : 등록 정보
// 49 Page : 축적 정보
//#define ADDR_FLASH_PAGE_48   ((uint32_t)0x08018000) /* Base address of Page 48, 2 Kbytes */
//#define ADDR_FLASH_PAGE_49   ((uint32_t)0x08018800) /* Base address of Page 49, 2 Kbytes */


#include "w25qxx.h"
#include "w25qxxConf.h"
#include "Internal_Flash.h"
#include "W25Q_Load_Data.h"

#include "Compile_Data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_lpuart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart4_tx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */

#define debug_mode	0

//array to store addresses
uint8_t CCU_Address_tmp[8] ;
uint8_t CCU_Address ;

// Group_Data[1260]
uint8_t Group_Data[Repeater_Header_Number +(Repeater_Number * Repeater_Data_Number) + Repeater_Crc_Number + Repeater_End_Number ];

//Array that stores system information to be reported to the parent
//Group_Rep_Group_Data[1260]
uint8_t  Rep_Group_Data[Repeater_Header_Number +(Repeater_Number * Repeater_Data_Number) + Repeater_Crc_Number + Repeater_End_Number];

uint8_t Pow_Bod_Data[Pow_Bod_Max_Number][Pow_Bod_Header_Number + Pow_Bod_Data_Number + Pow_Bod_CkSum_Number + Pow_Bod_End_Number];

uint8_t Rep_Pow_Bod_Data[Pow_Bod_Header_Number + Pow_Bod_Data_Number + Pow_Bod_CkSum_Number + Pow_Bod_End_Number];

//Array to store sub-250 system registration information
//연동 정보를 넣는다. 0: 미등록 , 1: 일반형 중계기, 2: 아이솔레이터형 중계기, 3: 단선단락 자동 감지형 중계기, 4:정온식, 5: 아이솔레이터형 정온식 , 6:광전식 , 7: 아이솔레이터형 광전식
uint8_t REPEATER_Regster[Repeater_Number];
uint8_t REPEATER_Regster_Num;

//Array to Possible Communicatin In Out Out : 0 , In : 1
uint8_t REPEATER_Possible[Repeater_Number];

//Array to store the charge information of the lower 250 systems
uint8_t REPEATER_Charge_Regster[4];

//Array to store accumulation time information for lower 64 systems, does not operate if time is 0
//Group_Charge_Time[9]
uint8_t REPEATER_Charge_Time[Group_Charge_Time_Number];

//Array to store sub-250 system Response information
uint8_t REPEATER_Response_Regster[Repeater_IN_OUT][Repeater_Number];

//Array Repeater accumulation setting
uint8_t REPEATER_Acc_Set_Data[Repeater_Number];
uint8_t REPEATER_Acc_Com_Data[Repeater_Number];

//Array CCU Infomation
uint8_t CCU_Infomation_Data[256];

//Define UI uart receiving array
uint8_t UI_UART_TX_buf[UI_UART_buf_len] ;
uint8_t UI_UART_RX_buf[UI_UART_buf_len];
uint8_t UI_UART_Tx_buf_tmp[UI_UART_buf_len];
uint8_t UI_UART_RX_buf_tmp[UI_UART_buf_len];
uint8_t UI_UART_State ;					// Define reception start state
int UI_UART_buf_count;					// Save the length of received data after starting reception
int UI_UART_buf_count_tmp;				// Variable to store the length of the received data before initializing it when reception is completed
uint8_t UI_UART_Receive_complete;		// Variable indicating that reception has been completed
int UI_UART_buf_count_Save;

uint8_t UART_TX_buf[2][UART_buf_len] ;
uint8_t UART_RX_buf[2][UART_buf_len] ;
uint8_t UART_RX_buf_tmp[2][UART_buf_len] ;
uint8_t UART_RX_buf_temp;
uint8_t UART_State[2];					// Define reception start state
int UART_buf_count[2];					// Save the length of received data after starting reception
int UART_buf_count_tmp[2];				// Variable to store the length of the received data before initializing it when reception is completed
uint8_t UART_Receive_complete[2];			// Variable indicating that reception has been completed
uint8_t UART_Transmit_complete[2];
uint8_t UART_DMA_CNT[2];					// Time check variable after DMA start
int UART_DMA_ERR_CNT[2];				//DMA error count

uint8_t UART2_TX_DMA_complete,UART4_TX_DMA_complete;

float Loop_In_Voltage;
float Loop_Out_Voltage;

uint8_t Loop_Back_Mode;
uint8_t Loop_Back_Start;
uint8_t Loop_Back_First;

uint8_t Loop_In_Status;
uint8_t Loop_Out_Status;

uint8_t Com_Out_Status;
uint8_t Com_In_Status;

uint32_t Flash_ID;
uint8_t Flash_Status;

uint8_t Reset_UI_uart, Reset_uart;
uint8_t REPEATER_Read_Regster[Repeater_Number];
uint8_t REPEATER_Final_Read_Regster;

uint8_t REPEATER_first_fire;
uint8_t REPEATER_In_Short[Repeater_Number][In_Short_Cnt];

uint8_t End_REPEATER_Number;
uint8_t End_Read_REPEATER_Number;
uint8_t End_Out_ISO_REPEATER_Number;
uint8_t End_In_ISO_REPEATER_Number;

uint8_t REPEATER_Version[Repeater_Number+1][Version_Cnt]; 	// REPEATER_Version[0]  :  ccu 버전 정보
uint8_t REPEATER_Version_Read[Repeater_Number];
uint8_t Rep_Version_Data[Rep_Version_Data_Number];

uint8_t Info_Data_Version[6];
//1200 * 12 = 14400
int UART_BAUDRATE, Read_Baud_val;

int CCU_Mode;


uint8_t Rep_ISO_In_Open;
uint8_t Rep_ISO_In_Short;
uint8_t Rep_ISO_Out_Open;
uint8_t Rep_ISO_Out_Short;
uint8_t In_Volt_St, Out_Vol_St;
uint8_t In_Volt_Val, Out_Vol_Val;
uint8_t Rep_In_Relay_Mode;
uint8_t Rep_Out_Relay_Mode;

uint8_t read_End_value;

uint8_t First_Boot = 0;

int cnt_tmp;
////////////////////////////

uint8_t Link_Table_Year_Month;
uint8_t Link_Table__Day;
uint8_t Link_Table_Hour;
uint8_t Serial_Num[7];

uint8_t In_Relay_Mode;
uint8_t Out_Relay_Mode;

uint8_t Boot_Mode;
uint8_t Loop_mode;

uint8_t Sub_Version_Read_Cnt;

uint8_t Sub_Set_Mode , Mini_Mode;
uint8_t Rep_Set_Mode;

uint8_t Analog_Test_Mode;

#define Analog_optic_Test_Mode	1
#define Analog_temp_Test_Mode	2
#define Def_Out_Mode			3

uint8_t Info_Data_Tmp;
uint8_t Read_Info_Mode;
#define Read_Info_Def_Mode	1
#define Read_Info_Test_Mode	2

uint8_t Fire_Data_tmp[Repeater_Number][4][Fire_Data_Num]; // 주소,입력 포트 , 해제 카운트
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_FLASH_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

///////////////Start timer interrupt operation function///////////////////

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1)
  {
    HAL_GPIO_TogglePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin);

    if(HAL_GPIO_ReadPin(COMM1_FS_GPIO_Port, COMM1_FS_Pin) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_TogglePin(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin);
    }

    if(HAL_GPIO_ReadPin(COMM2_FS_GPIO_Port, COMM2_FS_Pin) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(ERR_UART2_LED_GPIO_Port, ERR_UART2_LED_Pin, GPIO_PIN_SET);
    }
    else{
      HAL_GPIO_TogglePin(ERR_UART2_LED_GPIO_Port, ERR_UART2_LED_Pin);
    }

    HAL_IWDG_Refresh(&hiwdg);

  }
  else if(htim->Instance == TIM16)
  {
//	  HAL_GPIO_TogglePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin);
//	  if(Reset_uart == 1){
//
//		  MX_USART4_UART_Init();
//		  MX_USART3_UART_Init();
//
//		  /* huart3 RX Interrupt  Enable */
//		  /* Process Unlocked */
//		  __HAL_UNLOCK(&huart3);
//		  /* Enable the UART Parity Error Interrupt */
//		  __HAL_UART_ENABLE_IT(&huart3, UART_IT_PE);
//		  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//		  __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
//		  /* Enable the UART Data Register not empty Interrupt */
//		  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
//
//		  /* huart1 RX Interrupt  Enable */
//		  /* Process Unlocked */
//		  __HAL_UNLOCK(&huart4);
//		  /* Enable the UART Parity Error Interrupt */
//		  __HAL_UART_ENABLE_IT(&huart4, UART_IT_PE);
//		  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//		  __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
//		  /* Enable the UART Data Register not empty Interrupt */
//		  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
//	  }

	  if(Reset_UI_uart == 1){

		HAL_UART_DMAStop(&hlpuart1);
		HAL_UART_DeInit(&hlpuart1);
		MX_LPUART1_UART_Init();

		/* huart1 RX Interrupt  Enable */
		__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_ERR);
		/* Enable the UART Data Register not empty Interrupt */
		__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);

	  }
	  else{
		  Reset_UI_uart = 1;
	  }
  }

}

/**
  * @brief  TIM Period Elapsed Callback.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 인터럽트가 TIM7에서 발생했는지 확인
    if (htim->Instance == TIM6)
    {
    	LED_Control(UART2_RXD_LED_GPIO_Port, UART2_RXD_LED_Pin,LED_Off);
        HAL_TIM_Base_Stop_IT(&htim6);
    }

    // 인터럽트가 TIM7에서 발생했는지 확인
    if (htim->Instance == TIM7)
    {
    	LED_Control(UART1_RXD_LED_GPIO_Port, UART1_RXD_LED_Pin,LED_Off);
        HAL_TIM_Base_Stop_IT(&htim7);
    }
}
////////////////Timer interrupt operation function ends ///////////////////


///////////////Start UART interrupt operation function///////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	if (huart->Instance == LPUART1)
	{
	}

	else if (huart->Instance == USART2)
		{

		}

		else if (huart->Instance == USART4)
		{
		}

}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	///////////If the DMA UART is malfunctioning, change the reception mode to an interrupt./////
	if (huart->Instance == LPUART1)
	{
		//Rs485 Rx mode//
		/*
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

		LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_Off);
		Reset_UI_uart = 0;
*/
	}
	else if (huart->Instance == USART2)
	{
		UART_buf_count[1] = 0;
		Sub_Com_TX_LED_Off(2);
		HAL_GPIO_WritePin(UART2_TXD_EN_GPIO_Port, UART2_TXD_EN_Pin, GPIO_PIN_RESET);
		UART2_TX_DMA_complete = 0;
		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
		/* Enable the UART Data Register not empty Interrupt */
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	}
	else if (huart->Instance == USART4)
	{

		UART_buf_count[0] = 0;
		Sub_Com_TX_LED_Off(1);
		HAL_GPIO_WritePin(UART1_TXD_EN_GPIO_Port, UART1_TXD_EN_Pin, GPIO_PIN_RESET);
		UART4_TX_DMA_complete = 0;
		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
		/* Enable the UART Data Register not empty Interrupt */
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
	}
}


///////////////UART interrupt operation function terminated///////////////////


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
  MX_DMA_Init();
  MX_USART4_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_FLASH_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  Analog_Test_Mode = Def_Out_Mode;
//  Analog_Test_Mode = Analog_optic_Test_Mode;
//  Analog_Test_Mode = Analog_temp_Test_Mode;


// Read_Info_Mode = Read_Info_Test_Mode : 형슥승인용 모드
//  Read_Info_Mode = Read_Info_Def_Mode;
  Read_Info_Mode = Read_Info_Test_Mode;


  // Timer1: 1 second cycle
  // Timer16: 0.1 second cycle
  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);

  HAL_TIM_OC_Start_IT(&htim16,TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htim15);

  Compile_Date();

  REPEATER_Version[0][0] = F_Version_Year;
  REPEATER_Version[0][1] = F_Version_Month;
  REPEATER_Version[0][2] = F_Version_Day;
  REPEATER_Version[0][3] = F_Version_Hour;
  REPEATER_Version[0][4] = F_Version_Min;
  REPEATER_Version[0][5] = F_Version_Sec;

  Set_PWM(OUT_PORT , 300);
  Set_PWM(IN_PORT , 300);

  //Out Port Uart 1 -> Uart 4
  //In Port Uart 2 -> Uart 2
for(int i=0; i< 2; i++){
	  LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_On);
	  LED_Control(RUN_LED2_GPIO_Port, RUN_LED2_Pin , LED_On);

	  LED_Control(UART1_TXD_LED_GPIO_Port, UART1_TXD_LED_Pin , LED_On);
	  LED_Control(UART1_RXD_LED_GPIO_Port, UART1_RXD_LED_Pin , LED_On);
	  LED_Control(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin , LED_On);

	  LED_Control(UART2_TXD_LED_GPIO_Port, UART2_TXD_LED_Pin , LED_On);
	  LED_Control(UART2_RXD_LED_GPIO_Port, UART2_RXD_LED_Pin , LED_On);
	  LED_Control(ERR_UART2_LED_GPIO_Port, ERR_UART1_LED_Pin , LED_On);

	  //Rs485 Rx mode//
	  HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

	  HAL_Delay(200);

	  LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_Off);
	  LED_Control(RUN_LED2_GPIO_Port, RUN_LED2_Pin , LED_Off);

	  LED_Control(UART1_TXD_LED_GPIO_Port, UART1_TXD_LED_Pin , LED_Off);
	  LED_Control(UART1_RXD_LED_GPIO_Port, UART1_RXD_LED_Pin , LED_Off);
	  LED_Control(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin , LED_Off);

	  LED_Control(UART2_TXD_LED_GPIO_Port, UART2_TXD_LED_Pin , LED_Off);
	  LED_Control(UART2_RXD_LED_GPIO_Port, UART2_RXD_LED_Pin , LED_Off);
	  LED_Control(ERR_UART2_LED_GPIO_Port, ERR_UART2_LED_Pin , LED_Off);

	  HAL_Delay(200);
}


uint8_t Fire_Data_tmp[Repeater_Number][4][Fire_Data_Num]; // 주소,입력 포트 , 해제 카운트
for(int i=0; i<Repeater_Number ; i++){
	for(int j=0; j<4; j++){
		for(int k=0; k<Fire_Data_Num; k++){
			Fire_Data_tmp[i][j][k] = 0;
		}
	}
}


  if(HAL_GPIO_ReadPin(BOOT_MODE_GPIO_Port, BOOT_MODE_Pin) == GPIO_PIN_SET){
	  Boot_Mode = Latch_mode;
  }
  else{
	  Boot_Mode = Relay_mode;
  }


  ///////////////Start reading address of board//////////////////

	CCU_Address_tmp[0] =  ~(HAL_GPIO_ReadPin(DIP0_GPIO_Port, DIP0_Pin)) & 0x01;
	CCU_Address_tmp[1] =  ~(HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin)) & 0x01;
	CCU_Address_tmp[2] =  ~(HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin)) & 0x01;
	CCU_Address_tmp[3] =  ~(HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin)) & 0x01;
	CCU_Address_tmp[4] =  ~(HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin)) & 0x01;
	CCU_Address_tmp[5] =  ~(HAL_GPIO_ReadPin(DIP5_GPIO_Port, DIP5_Pin)) & 0x01;
	CCU_Address_tmp[6] =  ~(HAL_GPIO_ReadPin(DIP6_GPIO_Port, DIP6_Pin)) & 0x01;
	CCU_Address_tmp[7] =  ~(HAL_GPIO_ReadPin(DIP7_GPIO_Port, DIP7_Pin)) & 0x01;

	CCU_Address = (CCU_Address_tmp[3] << 3)|(CCU_Address_tmp[2] << 2)|(CCU_Address_tmp[1] << 1)|(CCU_Address_tmp[0] << 0);

	if((CCU_Address_tmp[3]) == 1){
		Mini_Mode = 1;
	}
	else{
		Mini_Mode = 0;
	}

	if((CCU_Address_tmp[4]) == 1){
		Rep_Set_Mode = Rep_Number_30;
	}
	else{
		Rep_Set_Mode = Rep_Number_220;
	}

	if(CCU_Address_tmp[5] == 1){
	  Loop_mode = Loop_Set;
	}
	else{
	  Loop_mode = Normal_Set;
	}

	Read_Baud_val =(CCU_Address_tmp[6] << 1)|(CCU_Address_tmp[7] << 0);

	//    1	:	2400
	//    2	:	4800
	//    3	:	7200
	//    4	:	9600
	//    5	:	12000
	//    6	:	14400
	//    7	:	16800
	//    (CCU_Address_tmp[7] << 2)|

	UART_BAUDRATE = 9600*0.5*(Read_Baud_val+2);

	if(CCU_Address > 8){
	  CCU_Mode = 1; // 1 Control , 0 Repeater
	  CCU_Address = CCU_Address - 8;
	}


  PLC_Enable(OUT_PORT,PLC_Off);
  PLC_Enable(IN_PORT,PLC_Off);

  Relay_Control(OUT_PORT, Realay_Off,5,1);
  Relay_Control(IN_PORT, Realay_Off,5,1);

  if(Loop_mode == Loop_Set){
	  HAL_Delay(4000);
  }
  else{
	  HAL_Delay(500);
  }

  if(CCU_Address == 0){
	  while(1){
		  Relay_Control(IN_PORT, Realay_On,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(500);
		  Relay_Control(IN_PORT, Realay_Off,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(500);

		  Relay_Control(OUT_PORT, Realay_On,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(500);
		  Relay_Control(OUT_PORT, Realay_Off,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(500);
	  }
  }
  else{
	  for(int i=0; i<2; i++){
		  Relay_Control(IN_PORT, Realay_On,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(100);
		  Relay_Control(IN_PORT, Realay_Off,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(100);

		  Relay_Control(OUT_PORT, Realay_On,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(100);
		  Relay_Control(OUT_PORT, Realay_Off,Latch_Hold_Time,Latch_Change_Num);
		  HAL_Delay(100);
	  }
  }

  if(Boot_Mode == Latch_mode){
	  Relay_Control(OUT_PORT, Realay_On,Latch_Hold_Time,1);
  }
  else{
	  Relay_Control(OUT_PORT, Realay_On,Latch_Hold_Time,1);
	  Relay_Control(IN_PORT, Realay_On,Latch_Hold_Time,1);
  }
  PLC_Enable(OUT_PORT,PLC_On);
  PLC_Enable(IN_PORT,PLC_On);



  if(Loop_mode == Loop_Set){
	  HAL_Delay(12000);
  }
  else{
	  HAL_Delay(500);

  }

  Check_Over_Current();

  ///////////////End reading address of board//////////////////

  Group_Data[0] = 0x53;
  Group_Data[1] = 0x54;
  Group_Data[2] = 0x6a;
  Group_Data[3] = 0x72;
  Group_Data[4] = 0xFA;
  Group_Data[5] = CCU_Address;
  for(int i=0;i<220;i++){
	  Group_Data[6 + i*5] = i+1;
  }
  Group_Data[Repeater_Data_Total-2] = 0x45;
  Group_Data[Repeater_Data_Total-1] = 0x44;

  Flash_ID= W25qxx_ReadID();
  if(_MANUFACTURER_ID == ((Flash_ID >> 16) & 0xff)){
	  Flash_Status = 1;
	  W25qxx_Init();
  }

//  Read_W25Q_Device_Info();

  if(Read_Info_Mode == Read_Info_Def_Mode){
	  Read_Info_Data();

	  if(Rep_Set_Mode == Rep_Number_30 ){
		  for(int i=0; i<220; i++){
			  REPEATER_Regster[i] = 0;
		  }
		  for(int i=0; i<10; i++){
			  REPEATER_Regster[i] = 1;
		  }
	  }
	  else if(Rep_Set_Mode == Rep_Number_220){
		  for(int i=0; i<220; i++){
			  if(REPEATER_Regster[i] == 0xff){
				  REPEATER_Regster[i] = 1;
			  }
		  }
	  }

  }
  else if(Read_Info_Mode == Read_Info_Test_Mode){
	  //Read_Test_Info_Data();

	  if(Rep_Set_Mode == Rep_Number_30 ){
		  for(int i=0; i<220; i++){
			  REPEATER_Regster[i] = 0;
		  }
		  for(int i=0; i<10; i++){
			  REPEATER_Regster[i] = 1;
		  }
	  }
	  else if(Rep_Set_Mode == Rep_Number_220){
		  for(int i=0; i<220; i++){
			  REPEATER_Regster[i] = 1;
		  }
	  }

	  //Address 1
	  REPEATER_Charge_Regster[0] = 0;	//Port1
	  REPEATER_Charge_Regster[1] = 0;	//Port2
	  REPEATER_Charge_Regster[2] = 1;	//Port3
	  REPEATER_Charge_Regster[3] = 0;	//Port4
	  Info_Data_Tmp = ((REPEATER_Charge_Regster[3]&0x01)<<7)|((REPEATER_Charge_Regster[2]&0x01)<<6)|((REPEATER_Charge_Regster[1]&0x01)<<5)|((REPEATER_Charge_Regster[0]&0x01)<<4)|(REPEATER_Regster[0]&0x0f);
	  REPEATER_Regster[0] = Info_Data_Tmp;

	  //Address 2
	  REPEATER_Charge_Regster[0] = 0;	//Port1
	  REPEATER_Charge_Regster[1] = 1;	//Port2
	  REPEATER_Charge_Regster[2] = 1;	//Port3
	  REPEATER_Charge_Regster[3] = 1;	//Port4
	  Info_Data_Tmp = ((REPEATER_Charge_Regster[3]&0x01)<<7)|((REPEATER_Charge_Regster[2]&0x01)<<6)|((REPEATER_Charge_Regster[1]&0x01)<<5)|((REPEATER_Charge_Regster[0]&0x01)<<4)|(REPEATER_Regster[1]&0x0f);
	  REPEATER_Regster[1] = Info_Data_Tmp;

	  //Address 3
	  REPEATER_Charge_Regster[0] = 0;	//Port1
	  REPEATER_Charge_Regster[1] = 0;	//Port2
	  REPEATER_Charge_Regster[2] = 1;	//Port3
	  REPEATER_Charge_Regster[3] = 0;	//Port4
	  Info_Data_Tmp = ((REPEATER_Charge_Regster[3]&0x01)<<7)|((REPEATER_Charge_Regster[2]&0x01)<<6)|((REPEATER_Charge_Regster[1]&0x01)<<5)|((REPEATER_Charge_Regster[0]&0x01)<<4)|(REPEATER_Regster[2]&0x0f);
	  REPEATER_Regster[2] = Info_Data_Tmp;

	  //Address 4
	  REPEATER_Charge_Regster[0] = 0;	//Port1
	  REPEATER_Charge_Regster[1] = 1;	//Port2
	  REPEATER_Charge_Regster[2] = 1;	//Port3
	  REPEATER_Charge_Regster[3] = 0;	//Port4
	  Info_Data_Tmp = ((REPEATER_Charge_Regster[3]&0x01)<<7)|((REPEATER_Charge_Regster[2]&0x01)<<6)|((REPEATER_Charge_Regster[1]&0x01)<<5)|((REPEATER_Charge_Regster[0]&0x01)<<4)|(REPEATER_Regster[3]&0x0f);
	  REPEATER_Regster[3] = Info_Data_Tmp;

	  //Address 5
	  REPEATER_Charge_Regster[0] = 1;	//Port1
	  REPEATER_Charge_Regster[1] = 1;	//Port2
	  REPEATER_Charge_Regster[2] = 1;	//Port3
	  REPEATER_Charge_Regster[3] = 1;	//Port4
	  Info_Data_Tmp = ((REPEATER_Charge_Regster[3]&0x01)<<7)|((REPEATER_Charge_Regster[2]&0x01)<<6)|((REPEATER_Charge_Regster[1]&0x01)<<5)|((REPEATER_Charge_Regster[0]&0x01)<<4)|(REPEATER_Regster[4]&0x0f);
	  REPEATER_Regster[4] = Info_Data_Tmp;

	  REPEATER_Regster[5] = 6;

	  REPEATER_Regster[6] = 4;
  }

  Read_Charge_Data();

/*
  REPEATER_Acc_Set_Data[0] = 0x5f;
  REPEATER_Acc_Set_Data[1] = 0x5f;
  REPEATER_Acc_Set_Data[2] = 0x5f;
  REPEATER_Acc_Set_Data[3] = 0x5f;
  REPEATER_Acc_Set_Data[4] = 0x5f;
*/

  Set_Acc_OnOff();

  /*
  uint8_t W25Q_Page_Read_t[512];
  uint8_t W25Q_Page_Write_t[512];

  for(int i=0; i<256; i++){
	  W25Q_Page_Read_t[i] = i;
  }

  Write_To_Flash(Info_In_ADDR_FLASH, W25Q_Page_Read_t,512);
  Write_To_Flash(Charge_In_ADDR_FLASH, W25Q_Page_Read_t,512);
*/
  //Test_Init();



/*
  uint8_t W25Q_Page_Read_t[512];
  uint8_t W25Q_Page_Write_t[512];

  for(int i=0; i<512; i++){
	  W25Q_Page_Read_t[i] = 0;
  }

  W25qxx_ReadBytes(W25Q_Page_Read_t, 0, 256);

  W25qxx_ReadBytes(W25Q_Page_Read_t, 0, 256);
  for(int i=0; i< 256 ; i++){
	  W25Q_Page_Write_t[i] = i;
    }

  W25qxx_EraseBlock(0);
  W25qxx_ReadPage(W25Q_Page_Read_t, 0,0, 256);
  W25qxx_WritePage(W25Q_Page_Write_t,0 ,0,256);


  W25qxx_ReadPage(W25Q_Page_Read_t, 0, 0, 256);
*/

  Loop_Back_Mode = Loop_Back_Disable;
  Loop_Back_Start = Loop_Back_Disable;
  Loop_Back_First = Loop_Back_Disable;

  Com_Out_Status = Com_On;
  Com_In_Status = Com_Off;


  /* huart1 RX Interrupt  Enable */
  /* Process Unlocked */
  __HAL_UNLOCK(&hlpuart1);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);

  /* huart2 RX Interrupt  Enable */
  /* Process Unlocked */
  __HAL_UNLOCK(&huart2);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  /* huart1 RX Interrupt  Enable */
  /* Process Unlocked */
  __HAL_UNLOCK(&huart4);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);


  if(Analog_Test_Mode == Analog_optic_Test_Mode){

	  for(int i=0; i<220; i++){
		  REPEATER_Regster[i] = 0;
	  }

	  for(int i=0; i<6; i++){
		  REPEATER_Regster[i] = 1;
	  }

	  while(1){
		  Analog_test_read();
		  HAL_Delay(200);
	  }
  }
  else if(Analog_Test_Mode == Analog_temp_Test_Mode){

	  for(int i=0; i<220; i++){
		  REPEATER_Regster[i] = 0;
	  }

	  for(int i=0; i<10; i++){
		  REPEATER_Regster[i] = 1;
	  }

	  while(1){
		  Analog_test_read();
		  HAL_Delay(200);
	  }

  }

  else if(Analog_Test_Mode == Def_Out_Mode){

  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_Delay(1000);

	/* huart1 RX Interrupt  Enable */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);

	/* huart3 RX Interrupt  Enable */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	/* huart1 RX Interrupt  Enable */
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);

	Check_UI_UART_Receive(UI_UART_Receive_complete);
	//Set_BAUDRATE(UART_BAUDRATE);
	Check_UI_UART_Receive(UI_UART_Receive_complete);
	//Read_Reapeter_Data();

	if(Loop_mode == Normal_Set){
		Check_UI_UART_Receive(UI_UART_Receive_complete);
		Set_BAUDRATE(UART_BAUDRATE,OUT_PORT);
		Check_UI_UART_Receive(UI_UART_Receive_complete);
		Read_Reapeter_Data(OUT_PORT);

	}
	else{
		if(Loop_Back_Start == Loop_Back_Disable){
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			Set_BAUDRATE(UART_BAUDRATE,OUT_PORT);
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			Read_Reapeter_Data(OUT_PORT);

			//Read_Num_Reapeter_Data(OUT_PORT,Repeater_Number);
		}
		else if(Loop_Back_Start == Loop_Back_Enable){
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			Set_BAUDRATE(UART_BAUDRATE,OUT_PORT);
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			Set_BAUDRATE(UART_BAUDRATE,IN_PORT);
			Check_UI_UART_Receive(UI_UART_Receive_complete);

			for(int i=0; i<3; i++){
				read_End_value =  Check_Line_Status(End_REPEATER_Number+1 , IN_PORT);
				if(REPEATER_Regster[End_Out_ISO_REPEATER_Number] == 2){
					Set_Relay(End_REPEATER_Number+1 ,IN_PORT,0,1);
				}
				else if((REPEATER_Regster[End_Out_ISO_REPEATER_Number] == 4)|(REPEATER_Regster[End_Out_ISO_REPEATER_Number] == 6)){
					Set_Relay(End_REPEATER_Number+1 ,IN_PORT,1,1);
				}

				HAL_Delay(100);
			}
			Read_Num_Reapeter_Data(OUT_PORT,End_Out_ISO_REPEATER_Number+1);
			Read_Num_Reapeter_Data(IN_PORT,End_Out_ISO_REPEATER_Number+1);
		}

		Check_Relay();
	}
/*
	Check_UI_UART_Receive(UI_UART_Receive_complete);
	if(Loop_mode == Loop_Set){
		Check_Relay();
	}
	*/
	Check_UI_UART_Receive(UI_UART_Receive_complete);

	Check_Over_Current();

	Check_UI_UART_Receive(UI_UART_Receive_complete);

	if(Sub_Version_Read_Cnt == 1){
		Sub_Version_Read_Cnt = 0;
		for(int i=0; i< 3; i++){
		  Read_Repeater_Version_All(OUT_PORT);
		  Check_UI_UART_Receive(UI_UART_Receive_complete);
		}
	}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 7;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FLASH Initialization Function
  * @param None
  * @retval None
  */
static void MX_FLASH_Init(void)
{

  /* USER CODE BEGIN FLASH_Init 0 */

  /* USER CODE END FLASH_Init 0 */

  /* USER CODE BEGIN FLASH_Init 1 */

  /* USER CODE END FLASH_Init 1 */
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FLASH_Lock() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FLASH_Init 2 */

  /* USER CODE END FLASH_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  hiwdg.Init.EWI = 0;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 1500000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 5599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 55;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 55;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 5599;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 89;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 5599;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 89;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 55;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 5599;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 29999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UART1_D1_Pin|UART1_D2_Pin|LOOP_OUT_OFF_SET_Pin|UART2_D1_Pin
                          |LOOP_OUT_ON_SET_Pin|UART2_D2_Pin|LOOP_IN_ON_SET_Pin|ERR_UART2_LED_Pin
                          |ERR_UART1_LED_Pin|UART2_RXD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MCU_PLC1_EN_Pin|UART1_TXD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_PLC2_EN_Pin|LOOP_IN_OFF_SET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UART2_TXD_EN_Pin|RS485_DE_Pin|RS485_RE_Pin|SPI_CS_Pin
                          |UART2_TXD_LED_Pin|UART1_TXD_LED_Pin|RUN_LED2_Pin|RUN_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART1_RXD_LED_GPIO_Port, UART1_RXD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UART1_D1_Pin UART1_D2_Pin LOOP_OUT_OFF_SET_Pin UART2_D1_Pin
                           LOOP_OUT_ON_SET_Pin UART2_D2_Pin LOOP_IN_ON_SET_Pin ERR_UART2_LED_Pin
                           ERR_UART1_LED_Pin UART2_RXD_LED_Pin */
  GPIO_InitStruct.Pin = UART1_D1_Pin|UART1_D2_Pin|LOOP_OUT_OFF_SET_Pin|UART2_D1_Pin
                          |LOOP_OUT_ON_SET_Pin|UART2_D2_Pin|LOOP_IN_ON_SET_Pin|ERR_UART2_LED_Pin
                          |ERR_UART1_LED_Pin|UART2_RXD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_PLC1_EN_Pin UART1_TXD_EN_Pin */
  GPIO_InitStruct.Pin = MCU_PLC1_EN_Pin|UART1_TXD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : COMM1_FS_Pin COMM2_FS_Pin */
  GPIO_InitStruct.Pin = COMM1_FS_Pin|COMM2_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_PLC2_EN_Pin LOOP_IN_OFF_SET_Pin */
  GPIO_InitStruct.Pin = MCU_PLC2_EN_Pin|LOOP_IN_OFF_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UART2_TXD_EN_Pin RS485_DE_Pin RS485_RE_Pin SPI_CS_Pin
                           UART2_TXD_LED_Pin UART1_TXD_LED_Pin RUN_LED2_Pin RUN_LED1_Pin */
  GPIO_InitStruct.Pin = UART2_TXD_EN_Pin|RS485_DE_Pin|RS485_RE_Pin|SPI_CS_Pin
                          |UART2_TXD_LED_Pin|UART1_TXD_LED_Pin|RUN_LED2_Pin|RUN_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP0_Pin DIP1_Pin DIP2_Pin DIP3_Pin */
  GPIO_InitStruct.Pin = DIP0_Pin|DIP1_Pin|DIP2_Pin|DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP4_Pin DIP5_Pin DIP6_Pin DIP7_Pin */
  GPIO_InitStruct.Pin = DIP4_Pin|DIP5_Pin|DIP6_Pin|DIP7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UART1_RXD_LED_Pin */
  GPIO_InitStruct.Pin = UART1_RXD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART1_RXD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_MODE_Pin */
  GPIO_InitStruct.Pin = BOOT_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOOT_MODE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void delay_us(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim15, 0);              // 타이머를 0으로 초기화:<

	while((__HAL_TIM_GET_COUNTER(&htim15))<time){
		//HAL_GPIO_TogglePin(SYS_LED1_GPIO_Port, SYS_LED1_Pin);// 설정한 시간까지 대기
	}
}

void LED_Control(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin , uint16_t Staus){

  if(Staus == LED_On){
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
  }
  else if(Staus == LED_Off){
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
  }
}

void Relay_Control(uint16_t Port, uint16_t Staus, uint16_t Wate_Time, uint16_t Latch_Num){

	if(Boot_Mode == Latch_mode){
		if(Port == IN_PORT){
			if(Staus == Realay_On){
				In_Relay_Mode = Realay_On;
				for(int i=0; i< Latch_Num ; i++){
					HAL_GPIO_WritePin(LOOP_IN_ON_SET_GPIO_Port, LOOP_IN_ON_SET_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LOOP_IN_OFF_SET_GPIO_Port, LOOP_IN_OFF_SET_Pin, GPIO_PIN_RESET);

					HAL_Delay(Wate_Time);

					HAL_GPIO_WritePin(LOOP_IN_ON_SET_GPIO_Port, LOOP_IN_ON_SET_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LOOP_IN_OFF_SET_GPIO_Port, LOOP_IN_OFF_SET_Pin, GPIO_PIN_RESET);
				}
			  }
			  else if(Staus == Realay_Off){
				In_Relay_Mode = Realay_Off;
				for(int i=0; i< Latch_Num ; i++){
					HAL_GPIO_WritePin(LOOP_IN_ON_SET_GPIO_Port, LOOP_IN_ON_SET_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LOOP_IN_OFF_SET_GPIO_Port, LOOP_IN_OFF_SET_Pin, GPIO_PIN_SET);

					HAL_Delay(Wate_Time);

					HAL_GPIO_WritePin(LOOP_IN_ON_SET_GPIO_Port, LOOP_IN_ON_SET_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LOOP_IN_OFF_SET_GPIO_Port, LOOP_IN_OFF_SET_Pin, GPIO_PIN_RESET);
				}
			  }
		}
		else if(Port == OUT_PORT){
			if(Staus == Realay_On){
				Out_Relay_Mode = Realay_On;
				for(int i=0; i< Latch_Num ; i++){
					HAL_GPIO_WritePin(LOOP_OUT_ON_SET_GPIO_Port, LOOP_OUT_ON_SET_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LOOP_OUT_OFF_SET_GPIO_Port, LOOP_OUT_OFF_SET_Pin, GPIO_PIN_RESET);

					HAL_Delay(Wate_Time);

					HAL_GPIO_WritePin(LOOP_OUT_ON_SET_GPIO_Port, LOOP_OUT_ON_SET_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LOOP_OUT_OFF_SET_GPIO_Port, LOOP_OUT_OFF_SET_Pin, GPIO_PIN_RESET);
				}
			}
		  else if(Staus == Realay_Off){
			In_Relay_Mode = Realay_Off;
			for(int i=0; i< Latch_Num ; i++){
				HAL_GPIO_WritePin(LOOP_OUT_ON_SET_GPIO_Port, LOOP_OUT_ON_SET_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LOOP_OUT_OFF_SET_GPIO_Port, LOOP_OUT_OFF_SET_Pin, GPIO_PIN_SET);

				HAL_Delay(Wate_Time);

				HAL_GPIO_WritePin(LOOP_OUT_ON_SET_GPIO_Port, LOOP_OUT_ON_SET_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LOOP_OUT_OFF_SET_GPIO_Port, LOOP_OUT_OFF_SET_Pin, GPIO_PIN_RESET);
			}
		  }
		}
	}
	else{
		if(Port == IN_PORT){
			if(Staus == Realay_On){
				In_Relay_Mode = Realay_On;
				HAL_GPIO_WritePin(LOOP_IN_ON_SET_GPIO_Port, LOOP_IN_ON_SET_Pin, GPIO_PIN_RESET);
			  }
			  else if(Staus == Realay_Off){
				In_Relay_Mode = Realay_Off;
				HAL_GPIO_WritePin(LOOP_IN_ON_SET_GPIO_Port, LOOP_IN_ON_SET_Pin, GPIO_PIN_SET);
			  }
		}
		else if(Port == OUT_PORT){
			if(Staus == Realay_On){
				Out_Relay_Mode = Realay_On;
				HAL_GPIO_WritePin(LOOP_OUT_ON_SET_GPIO_Port, LOOP_OUT_ON_SET_Pin, GPIO_PIN_RESET);
			  }
			  else if(Staus == Realay_Off){
				Out_Relay_Mode = Realay_Off;
				HAL_GPIO_WritePin(LOOP_OUT_ON_SET_GPIO_Port, LOOP_OUT_ON_SET_Pin, GPIO_PIN_SET);
			  }
		}
	}
}
void Set_PWM(int G_num , int pwm_val){

  if(G_num == IN_PORT){
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  }
  else if (G_num == OUT_PORT){
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  }

  if(pwm_val < 1){
	  pwm_val = 1;
  }
  else {

    if(pwm_val > 1000){
      pwm_val = 1000;
    }

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Init.Prescaler = 55;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;


    if(G_num == IN_PORT){
    	htim3.Instance = TIM2;
		sConfigOC.Pulse = pwm_val;
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
		Error_Handler();
		}
		HAL_TIM_MspPostInit(&htim2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }
    else if(G_num == OUT_PORT){
		htim3.Instance = TIM3;
		sConfigOC.Pulse = pwm_val;
		if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
		Error_Handler();
		}
		HAL_TIM_MspPostInit(&htim3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    }


  }

}



void Read_Num_Reapeter_Data(uint8_t Com_Status,uint8_t Read_Num){

	int Setart_Num = 0, End_Num = 0;

	if(Com_Status == OUT_PORT){
		Setart_Num = 0;
		End_Num = Read_Num;
	}
	else if(Com_Status == IN_PORT){
		Setart_Num = Read_Num;
		End_Num = Repeater_Number;
	}

	for(int i = Setart_Num ; i<End_Num ; i++){
		REPEATER_Read_Regster[i] = 0;
		if(REPEATER_Regster[i] != 0){
			for(int j=0; j< Read_Sub_Cnt ; j++){
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				Set_PWM(Com_Status , 300 - (j*50));
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				Sub_Send_Command(Com_Status,i+1,0xC0,0x00);
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				for(int j=0; j<Max_Wate_Recever ;j++){
					if(UART_Receive_complete[Com_Status] == 1){
						break;
					}
					delay_us(Max_Wate_udelay);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
				}
				Check_UART_Receive(Com_Status, UART_Receive_complete[Com_Status]);
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				if(UART_Receive_complete[Com_Status] == 1){
					REPEATER_Read_Regster[i] = 1;
					break;
				}
			}
			if(REPEATER_first_fire == 1){
				for(int j=0; j< Read_Sub_Cnt ; j++){
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					Set_PWM(Com_Status , 300 - (j*50));
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					Sub_Send_Command(Com_Status,i+1,0xC0,0x00);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					for(int j=0; j<Max_Wate_Recever ;j++){
						if(UART_Receive_complete[Com_Status] == 1){
							break;
						}
						delay_us(Max_Wate_udelay);
						Check_UI_UART_Receive(UI_UART_Receive_complete);
					}
					Check_UART_Receive(Com_Status, UART_Receive_complete[Com_Status]);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					if(UART_Receive_complete[Com_Status] == 1){
						break;

					}
				}
			}

		}
	}
}

void Read_Reapeter_Single_Data(uint8_t Address, uint8_t Com_Status){
	REPEATER_Read_Regster[Address] = 0;
	if(REPEATER_Regster[Address] != 0){
		for(int j=0; j< Read_Sub_Cnt ; j++){
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			Set_PWM(Com_Status , 300 - (j*50));
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			Sub_Send_Command(Com_Status,Address+1,0xC0,0x00);
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			for(int j=0; j<Max_Wate_Recever ;j++){
				if(UART_Receive_complete[Com_Status] == 1){
					break;
				}
				delay_us(Max_Wate_udelay);
				Check_UI_UART_Receive(UI_UART_Receive_complete);
			}
			Check_UART_Receive(Com_Status, UART_Receive_complete[Com_Status]);
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			if(UART_Receive_complete[Com_Status] == 1){
				REPEATER_Read_Regster[Address] = 1;
				break;
			}
		}
		if(REPEATER_first_fire == 1){
			for(int j=0; j< Read_Sub_Cnt ; j++){
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				Set_PWM(Com_Status , 300 - (j*50));
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				Sub_Send_Command(Com_Status,Address+1,0xC0,0x00);
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				for(int j=0; j<Max_Wate_Recever ;j++){
					if(UART_Receive_complete[Com_Status] == 1){
						break;
					}
					delay_us(Max_Wate_udelay);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
				}
				Check_UART_Receive(Com_Status, UART_Receive_complete[Com_Status]);
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				if(UART_Receive_complete[Com_Status] == 1){
					break;

				}
			}
		}

	}
}
void Set_BAUDRATE(int BAUDRATE, int Port){

	UART_ReInit(9600);
	HAL_Delay(10);
	Sub_Send_Command(Port,253,0xB0,UART_BAUDRATE/1200/2);
	HAL_Delay(10);
	UART_ReInit(UART_BAUDRATE);
	HAL_Delay(10);


}

uint8_t Read_Repeater_Version(int num, uint8_t Com_Status){

	uint8_t error = 1;

	UART_Receive_complete[Com_Status] = 0;
	Sub_Send_Command(Com_Status,num,0xB1,0x00);
	Check_UI_UART_Receive(UI_UART_Receive_complete);
	for(int j=0; j<Max_Wate_Recever ;j++){
		if(UART_Receive_complete[Com_Status] == 1){
			break;
		}
		delay_us(Max_Wate_udelay);
		Check_UI_UART_Receive(UI_UART_Receive_complete);
	}


	REPEATER_Version[num][0] = UART_RX_buf[Com_Status][2];
	REPEATER_Version[num][1] = UART_RX_buf[Com_Status][3];
	REPEATER_Version[num][2] = UART_RX_buf[Com_Status][4];

	if(UART_Receive_complete[Com_Status] == 0){
		error = 0;
	}

	for(int i=0 ; i<UART_buf_len; i++ ){
		UART_RX_buf[Com_Status][i] = 0;
	}

	UART_Receive_complete[Com_Status] = 0;
	Sub_Send_Command(Com_Status,num,0xB1,0x01);
	Check_UI_UART_Receive(UI_UART_Receive_complete);
	for(int j=0; j<Max_Wate_Recever ;j++){
		if(UART_Receive_complete[Com_Status] == 1){
			break;
		}
		delay_us(Max_Wate_udelay);
		Check_UI_UART_Receive(UI_UART_Receive_complete);
	}

	REPEATER_Version[num][3] = UART_RX_buf[Com_Status][2];
	REPEATER_Version[num][4] = UART_RX_buf[Com_Status][3];
	REPEATER_Version[num][5] = UART_RX_buf[Com_Status][4];

	if(UART_Receive_complete[Com_Status] == 0){
		error = 0;
	}

	for(int i=0 ; i<UART_buf_len; i++ ){
		UART_RX_buf[Com_Status][i] = 0;
	}

	return error;

}

void Read_Repeater_Version_All(uint8_t Com_Status){
	for(int i=0; i<Repeater_Number ; i++){
		if(REPEATER_Regster[i] == 1){
			if(REPEATER_Version_Read[i] == 0){
				REPEATER_Version_Read[i] = Read_Repeater_Version(i+1, Com_Status);
			}
		}
	}
}


void UART_ReInit(int BAUDRATE)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = BAUDRATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  huart4.Instance = USART4;
  huart4.Init.BaudRate = BAUDRATE;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

void PLC_Enable(int Loop, int Enable){

	if(Loop ==IN_PORT){
		if(Enable == PLC_On){
			HAL_GPIO_WritePin(UART2_D1_GPIO_Port, UART2_D1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(UART2_D2_GPIO_Port, UART2_D2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MCU_PLC2_EN_GPIO_Port, MCU_PLC2_EN_Pin, GPIO_PIN_RESET);
		}
		else if(Enable == PLC_Off){
			HAL_GPIO_WritePin(UART2_D1_GPIO_Port, UART2_D1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(UART2_D2_GPIO_Port, UART2_D2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MCU_PLC2_EN_GPIO_Port, MCU_PLC2_EN_Pin, GPIO_PIN_SET);
		}
	}
	else if(Loop ==OUT_PORT){
		if(Enable == PLC_On){
			HAL_GPIO_WritePin(UART1_D1_GPIO_Port, UART1_D1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(UART1_D2_GPIO_Port, UART1_D2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MCU_PLC1_EN_GPIO_Port, MCU_PLC1_EN_Pin, GPIO_PIN_RESET);
		}
		else if(Enable == PLC_Off){
			HAL_GPIO_WritePin(UART1_D1_GPIO_Port, UART1_D1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(UART1_D2_GPIO_Port, UART1_D2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MCU_PLC1_EN_GPIO_Port, MCU_PLC1_EN_Pin, GPIO_PIN_SET);
		}
	}
}

void Check_Over_Current(void){


    if(HAL_GPIO_ReadPin(COMM1_FS_GPIO_Port, COMM1_FS_Pin) == GPIO_PIN_SET){

    }
    else{
		for(int i=0; i<300; i++){
			PLC_Enable(OUT_PORT,PLC_Off);
			Check_UI_UART_Receive(UI_UART_Receive_complete);
			HAL_Delay(10);
		}
		PLC_Enable(OUT_PORT,PLC_On);
	}

    if(Loop_Back_Start == Loop_Back_Enable){
		if(HAL_GPIO_ReadPin(COMM2_FS_GPIO_Port, COMM2_FS_Pin) == GPIO_PIN_SET){

		}
		else{
			for(int i=0; i<300; i++){
				PLC_Enable(IN_PORT,PLC_Off);
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				HAL_Delay(10);
			}
			PLC_Enable(IN_PORT,PLC_On);
		}
    }

}

uint8_t Check_Line_Status(int num , uint8_t Com_Status){

	uint8_t Return_val = 0 ;

	for(int j=0; j< Read_Sub_Cnt ; j++){
		Check_UI_UART_Receive(UI_UART_Receive_complete);
		Set_PWM(Com_Status , 300 - (j*50));
		Check_UI_UART_Receive(UI_UART_Receive_complete);
		Sub_Send_Command(Com_Status,num,0xC5,0x00);
		Check_UI_UART_Receive(UI_UART_Receive_complete);
		for(int j=0; j<Max_Wate_Recever ;j++){
			if(UART_Receive_complete[Com_Status] == 1){
				break;
			}
			HAL_Delay(10);
			Check_UI_UART_Receive(UI_UART_Receive_complete);
		}
		//Check_UART_Receive(OUT_PORT, UART_Receive_complete[OUT_PORT]);
		Return_val = Check_Line_Data(UART_Receive_complete[Com_Status]);
		Check_UI_UART_Receive(UI_UART_Receive_complete);
		if(UART_Receive_complete[Com_Status] == 1){
			break;
		}
	}

	if(UART_Receive_complete[Com_Status] == 0){
		Return_val = 0xff;
	}
	else{
		Rep_ISO_In_Open = (Return_val >> 7) & 0x01;
		Rep_ISO_In_Short = (Return_val >> 6) & 0x01;
		Rep_ISO_Out_Open = (Return_val >> 5) & 0x01;
		Rep_ISO_Out_Short = (Return_val >> 4) & 0x01;
		In_Volt_St = (Return_val >> 3) & 0x01;
		Out_Vol_St = (Return_val >> 2) & 0x01;
		Rep_In_Relay_Mode = (Return_val >> 1) & 0x01;
		Rep_Out_Relay_Mode = (Return_val >> 0) & 0x01;
	}

	return Return_val;
}

uint8_t Check_Line_Data(int check_val){

	uint8_t Uart_crc = 0 ;
	uint8_t Uart_temp_data[32];

	if(check_val == 1){

		for(int i=0; i< sizeof(Uart_temp_data) ; i++){
			Uart_temp_data[i] = UART_RX_buf[0][i];
		}

		Uart_crc = Uart_temp_data[1]^Uart_temp_data[2]^Uart_temp_data[3]^Uart_temp_data[4];
		if(Uart_crc  == Uart_temp_data[5]){

		}

	}
	In_Volt_Val = Uart_temp_data[3];
	Out_Vol_Val = Uart_temp_data[4];

	return Uart_temp_data[2];

//	0b0??? ???? : 입력회로 정상
//	0b1??? ???? : 입력회로 단선
//	0b?0?? ???? : 입력회로 정상
//	0b?1?? ???? : 입력회로 단락
//	0b??0? ???? : 출력회로 정상
//	0b??1? ???? : 출력회로 단선
//	0b???0 ???? : 출력회로 정상
//	0b???1 ???? : 출력회로 단락
//	0b???? 1??? : 입력 전압 상태 on
//	0b???? 0??? : 입력 전압 상태 off
//	0b???? ?1?? :출력 전압 상태 on
//	0b???? ?0?? : 출력 전압 상태 off
//	0b???? ??1? : 입력 릴레 상태 on
//	0b???? ??0? : 입력 릴레이 상태 off
//	0b???? ???1 :출력 릴레이 상태 on
//	0b???? ???0 : 출력 릴레이 상태 off

}

void Set_Relay(uint8_t num,uint8_t Port ,uint8_t in_status, uint8_t out_status ){

	uint8_t Send_Data;

	Send_Data = ((in_status&0x01) << 1)|((out_status&0x01) << 0);
	Sub_Send_Command(Port,num,0xC6,Send_Data);

}

int Read_Start_Cnt = 0, Read_Err_Cnt = 0;

void Check_Relay(void){

	uint8_t read_temp;

	if(Loop_Back_Mode == Loop_Back_Disable){

		if(REPEATER_Read_Regster[End_REPEATER_Number] == 1){

			//Set_Relay(End_REPEATER_Number + 1,OUT_PORT,0,0);
			Loop_Back_Mode  = Loop_Back_Enable;

		}
		else{
			if(Loop_Back_Mode  == Loop_Back_Enable){
				if(First_Boot == 0){
					First_Boot = 1;
					PLC_Enable(OUT_PORT,PLC_Off);
					for(int j=0; j<500; j++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						HAL_Delay(10);
					}
					PLC_Enable(OUT_PORT,PLC_On);
					for(int j=0; j<1500; j++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						HAL_Delay(10);
					}
				}
			}

			else{
				if(Read_Err_Cnt < 2 ){
					PLC_Enable(OUT_PORT,PLC_Off);
					for(int j=0; j<500; j++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						HAL_Delay(10);
					}
					PLC_Enable(OUT_PORT,PLC_On);
					for(int j=0; j<1000; j++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						Check_Over_Current();
						HAL_Delay(10);
					}
					Read_Err_Cnt++;
				}
				else{

				}
			}

		}

	}
	else if((Loop_Back_Mode == Loop_Back_Enable)&(Loop_Back_Start == Loop_Back_Disable)){

		if(Loop_Back_First == Loop_Back_Disable){
			//마지막 중계기나 아날로그가 통신이 실패 하면 PLC 를 2번 리셋해 단락된 회선의 중게기를 off로 동작시킨다.
			if(REPEATER_Read_Regster[End_REPEATER_Number] == 0){

				// 마지막 중계기 읽기 실패하면 마지막 중계기만 10회 읽어 확인 한다.
				for(int i = 0; i< 10; i++){
					HAL_Delay(20);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					Read_Reapeter_Single_Data(End_REPEATER_Number, OUT_PORT);
					if(REPEATER_Read_Regster[End_REPEATER_Number] == 0){
						REPEATER_Final_Read_Regster++;
					}
					else{
						REPEATER_Final_Read_Regster = 0;
						break;
					}
				}


				// 마지막 중계기 읽기 실패 3회 이상시
				if(REPEATER_Final_Read_Regster > 4){
					//중계기가 전원 off시 프래쉬에 전원 off저장되어 한번씩은 재부팅 해야함



					// 5초동안  전원을 끈다. 일반 중계기일 경우 단락시 드라이버 전원이 재시작 할수 있고 아날로그 일경우에는 재시작 안할 가능성이 있다.
					// 중계기일경우 초기 부팅시 플래쉬 삭제로 라인 연결이 잘되나 아날로그일 경우 1회 재부팅 해줘야 하는 경우가 생긴다.
					// 따라서 아날로그 일경우 마지막 아날로그를 재부팅 해준다.
					PLC_Enable(OUT_PORT,PLC_Off);
					for(int i=0; i<500; i++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						HAL_Delay(10);
					}

					//
					PLC_Enable(OUT_PORT,PLC_On);
					HAL_Delay(100);
					for(int i=0; i<1500; i++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						Check_Over_Current();
						HAL_Delay(10);
					}


					// 응답한 중계기를 확인하는 레지스터를 초기화 한다.

					for(int j=0; j<Repeater_Number ; j++){
						REPEATER_Read_Regster[j] = 0;
					}

					for(int i=0; i<5 ; i++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						Set_BAUDRATE(UART_BAUDRATE,OUT_PORT);

						for(int j=0; j<Repeater_Number ; j++){
							Check_UI_UART_Receive(UI_UART_Receive_complete);
							Read_Reapeter_Single_Data(j, OUT_PORT);

							if(j > 10){
								cnt_tmp = 0;
								for(int k=0; k<10; k++){
									cnt_tmp += REPEATER_Read_Regster[j-k];
								}
								if(cnt_tmp ==0){
									break;
								}
							}
						}
					}
					//아웃풋 으로 읽은 마지막 중계기 찾기
					for(int i=0; i<220; i++){
					  if(REPEATER_Read_Regster[i] != 0){
						  End_Read_REPEATER_Number= i;
					  }
					}

					//아웃풋으로 읽을수 있는 마지막 아이솔레이션 중계기 찾기

					for(int i=0; i<(End_Read_REPEATER_Number + 1); i++){
					  if((REPEATER_Regster[i] == 2)|(REPEATER_Regster[i] == 4)|(REPEATER_Regster[i] == 6)){
						  End_Out_ISO_REPEATER_Number= i;
					  }
					}

					// 마지막 아이솔레이터의 중계기 정보 확인
					for(int i=0; i<5 ; i++){
						HAL_Delay(100);
						read_temp = Check_Line_Status(End_Out_ISO_REPEATER_Number + 1, OUT_PORT);
						if(read_temp == 0xff){

						}
						else{
							break;
						}
					}


					// 단선, 단락 일때 마지막 아이솔레이터 릴레이 끄기
					if(REPEATER_Regster[End_Out_ISO_REPEATER_Number] == 2){

						if(Rep_ISO_Out_Short == 1){

						}
						else if(Rep_ISO_Out_Open == 1){
							for(int i=0; i<3 ; i++){
								HAL_Delay(100);
								Set_Relay(End_Out_ISO_REPEATER_Number+1 ,OUT_PORT,0,0);
							}

						}
					}
					else if((REPEATER_Regster[End_Out_ISO_REPEATER_Number] == 4)|(REPEATER_Regster[End_Out_ISO_REPEATER_Number] == 6)){
						if((In_Volt_St  == 1)&(Out_Vol_St == 1)){
							for(int i=0; i<3 ; i++){
								HAL_Delay(100);
								Set_Relay(End_Out_ISO_REPEATER_Number+1 ,OUT_PORT,1,0);
							}
						}
						else if((In_Volt_St  == 1)&(Out_Vol_St == 0)){
							HAL_Delay(100);
							Set_Relay(End_Out_ISO_REPEATER_Number+1 ,OUT_PORT,1,0);
						}
						else if((In_Volt_St  == 0)&(Out_Vol_St == 1)){
							HAL_Delay(100);
							Set_Relay(End_Out_ISO_REPEATER_Number+1 ,OUT_PORT,0,1);
						}
					}

					// 인 루프백 실행

					PLC_Enable(IN_PORT,PLC_On);
					Relay_Control(OUT_PORT, Realay_On,5,1);
					Relay_Control(IN_PORT, Realay_On,5,1);

					for(int i=0; i<3; i++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						Set_BAUDRATE(UART_BAUDRATE,IN_PORT);
						read_End_value =  Check_Line_Status(End_REPEATER_Number+1 , IN_PORT);
						Set_Relay(End_REPEATER_Number+1 ,IN_PORT,1,1);
						HAL_Delay(100);
					}

					for(int i =0; i<10; i++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						Set_BAUDRATE(UART_BAUDRATE,OUT_PORT);
						Read_Num_Reapeter_Data(OUT_PORT,End_Out_ISO_REPEATER_Number+1);
						Check_Over_Current();
					}



					//인 루프 단락 확인

					PLC_Enable(IN_PORT,PLC_Off);
					Loop_Back_First = Loop_Back_Enable;
					Loop_Back_Start = Loop_Back_Enable;

					for(int i=0; i<500; i++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						HAL_Delay(10);
					}

					PLC_Enable(IN_PORT,PLC_On);
					HAL_Delay(100);

					for(int i=0; i<3; i++){
						Check_UI_UART_Receive(UI_UART_Receive_complete);
						Set_BAUDRATE(UART_BAUDRATE,IN_PORT);
						read_End_value =  Check_Line_Status(End_REPEATER_Number+1 , IN_PORT);
						Set_Relay(End_REPEATER_Number+1 ,IN_PORT,1,1);
						HAL_Delay(100);
					}

					for(int i=0; i<Repeater_Number ; i++){
						if(i<End_Out_ISO_REPEATER_Number){
							REPEATER_Possible[i] = 0;
						}
						else{
							REPEATER_Possible[i] = 1;

						}

					}
					REPEATER_Final_Read_Regster = 0;
				}
				else{

					REPEATER_Final_Read_Regster++;
				}
			}
			else{
				REPEATER_Final_Read_Regster = 0;
			}

		}
	}
}


void Read_Test_Info_Data(void){

	uint8_t Read_tmp_data[456];

	Flash_To_Read(Info_In_ADDR_FLASH, Read_tmp_data, 456);

	for(int i=0; i< 6; i++){
		Info_Data_Version[i] = Read_tmp_data[6+i];
	}

	for(int i=0; i<220; i++){
		REPEATER_Regster[i] = Read_tmp_data[(i*2)+13];
	}

}



void ReSet_UART1_IO(void){

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void Set_UART1_IO(void){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

   __HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART4 GPIO Configuration
	PA0     ------> USART4_TX
	PA1     ------> USART4_RX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

//REPEATER_Read_Regster[Repeater_Number]
void Read_Reapeter_Data(uint8_t Com_Status){

	for(int i = 0 ; i<Repeater_Number ; i++){
		REPEATER_Read_Regster[i] = 0;
		if((REPEATER_Regster[i]&0x0f) > 0){
			for(int j=0; j< Read_Sub_Cnt ; j++){
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				Set_PWM(Com_Status , 300 - (j*50));
				Check_UI_UART_Receive(UI_UART_Receive_complete);
				Sub_Send_Command(Com_Status,i+1,0xC0,0x00);
				Check_UI_UART_Receive(UI_UART_Receive_complete);

				for(int j=0; j<Max_Wate_Recever ;j++){
					if(UART_Receive_complete[Com_Status] == 1){
						REPEATER_Read_Regster[i] = 1;
						break;
					}
					delay_us(Max_Wate_udelay);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
				}

				Check_UI_UART_Receive(UI_UART_Receive_complete);

				Check_UART_Receive(Com_Status, REPEATER_Read_Regster[i]);
				if(REPEATER_Read_Regster[i] == 1){
					//Check_UART_Receive(Com_Status, UART_Receive_complete[Com_Status]);

					break;
				}
			}
			/*
			if(REPEATER_first_fire == 1){
				for(int j=0; j< Read_Sub_Cnt ; j++){
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					Set_PWM(Com_Status , 300 - (j*50));
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					Sub_Send_Command(Com_Status,i+1,0xC0,0x00);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					for(int j=0; j<Max_Wate_Recever ;j++){
						if(UART_Receive_complete[Com_Status] == 1){
							break;
						}
						delay_us(Max_Wate_udelay);
						Check_UI_UART_Receive(UI_UART_Receive_complete);
					}
					Check_UART_Receive(Com_Status, UART_Receive_complete[Com_Status]);
					Check_UI_UART_Receive(UI_UART_Receive_complete);
					if(UART_Receive_complete[Com_Status] == 1){
						break;

					}
				}
			}
			*/

		}
	}
}

void Read_Reapeter_Set_Data(uint8_t Com_Status, uint8_t Address){

	REPEATER_Read_Regster[Address - 1] = 0;
	Sub_Send_Command(Com_Status,Address,0xC0,0x00);
	for(int j=0; j<Max_Wate_Recever ;j++){
		if(UART_Receive_complete[Com_Status] == 1){
			REPEATER_Read_Regster[Address - 1] = 1;
			break;
		}
		delay_us(Max_Wate_udelay);
	}
	Check_UART_Receive(Com_Status, REPEATER_Read_Regster[Address - 1]);
}

void Analog_test_read(void){

	UART_BAUDRATE = 9600;

	/* huart1 RX Interrupt  Enable */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);

	/* huart3 RX Interrupt  Enable */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	/* huart1 RX Interrupt  Enable */
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);

	Check_UI_UART_Receive(UI_UART_Receive_complete);
	Read_Reapeter_Data(OUT_PORT);

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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
