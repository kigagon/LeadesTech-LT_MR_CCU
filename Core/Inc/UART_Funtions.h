/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UART_Funtions.h
  * @brief          : UART_Funtions
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
void UI_Receiver(void);
void IT_CCUs(UART_HandleTypeDef *huart, uint8_t PortNum);
void Check_UI_UART_Receive(int check_val);
void UI_Com_Func(void);
void UI_Com_Func_A(void);
void UI_Com_Func_S(void);
void UI_Com_Func_Reset(void);
void UI_Com_Func_Acc_OnOff(void);
void UI_Com_Func_Acc_Set(void);
void UI_Cmd_Func_P(void);
void UI_Com_Func_V(void);
void Run_UI_Com(void);

void UI_Com_Func_Infor_set(void);

void UI_Com_Func_S_Re(uint8_t Num);
void UI_Com_Func_Acc_Set_Re(uint8_t Num);

void UI_Com_Func_Anal_LED_Set(uint8_t Num);
