/*
 * Sub_Com.h
 *
 *  Created on: Sep 12, 2024
 *      Author: kiga
 */

#ifndef INC_SUB_COM_H_
#define INC_SUB_COM_H_



#endif /* INC_SUB_COM_H_ */

void Sub_Send_Command(uint8_t Send_Port, uint8_t Address, uint8_t Command, uint8_t Data0);

void Sub_Com_TX_LED_On(uint8_t LED_Num);
void Sub_Com_TX_LED_Off(uint8_t LED_Num);
void Sub_Com_RX_LED_On(uint8_t LED_Num);
void Sub_Com_RX_LED_Off(uint8_t LED_Num);
void Sub_Com_ST_LED_On(uint8_t LED_Num);
void Sub_Com_ST_LED_Off(uint8_t LED_Num);

void Sub_Com_RX_LED_Off_Start(uint8_t LED_Num);

void Sub_Com_TX_UART(uint8_t UART_Num,  const uint8_t *Uart_Tx_Data);
void Check_UART_Receive(int Port, int check_val);
void Check_Repeater_Charge(uint8_t num);

void Set_Acc_OnOff(void);
