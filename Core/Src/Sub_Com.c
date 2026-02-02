/*
 * Sub_Com.c
 *
 *  Created on: Sep 12, 2024
 *      Author: kiga
 */


#include "main.h"
#include "Sub_Com.h"
#include "Analog_Process.h"

void Sub_Send_Command(uint8_t Send_Port, uint8_t Address, uint8_t Command, uint8_t Data0){

	uint8_t Check_Sum, Send_Data_Length;

	UART_TX_buf[Send_Port][0] = 0x02;
	UART_TX_buf[Send_Port][1] = Address;
	UART_TX_buf[Send_Port][2] = Command;
	UART_TX_buf[Send_Port][3] = Data0;

	Check_Sum = UART_TX_buf[Send_Port][1]^UART_TX_buf[Send_Port][2]^UART_TX_buf[Send_Port][3];
	UART_TX_buf[Send_Port][4] = Check_Sum;
	UART_TX_buf[Send_Port][5] = 0x03;
	Send_Data_Length = 6;

	if(Send_Port == OUT_PORT){

		//Sub_Com_RX_LED_Off(1);
		Sub_Com_TX_LED_On(1);
		HAL_GPIO_WritePin(UART1_TXD_EN_GPIO_Port, UART1_TXD_EN_Pin, GPIO_PIN_SET);
		UART_Receive_complete[0] = 0;
		UART4_TX_DMA_complete = 1;
		HAL_UART_Transmit_DMA(&huart4, UART_TX_buf[Send_Port], Send_Data_Length);
		while(UART4_TX_DMA_complete);

		//Sub_Com_TX_LED_Off(1);
	}
	else if(Send_Port == IN_PORT){

		//Sub_Com_RX_LED_Off(2);
		Sub_Com_TX_LED_On(2);
		HAL_GPIO_WritePin(UART2_TXD_EN_GPIO_Port, UART2_TXD_EN_Pin, GPIO_PIN_SET);
		UART_Receive_complete[1] = 0;
		UART2_TX_DMA_complete = 1;
		HAL_UART_Transmit_DMA(&huart2, UART_TX_buf[Send_Port], Send_Data_Length);
		//Sub_Com_TX_LED_Off(2);
		while(UART2_TX_DMA_complete);
	}
}


void Sub_Com_TX_LED_On(uint8_t LED_Num){
	if(LED_Num == 1){
		LED_Control(UART1_TXD_LED_GPIO_Port, UART1_TXD_LED_Pin , LED_On);
	}
	else if(LED_Num == 2){
		LED_Control(UART2_TXD_LED_GPIO_Port, UART2_TXD_LED_Pin , LED_On);
	}
}

void Sub_Com_TX_LED_Off(uint8_t LED_Num){
	if(LED_Num == 1){
		LED_Control(UART1_TXD_LED_GPIO_Port, UART1_TXD_LED_Pin , LED_Off);
	}
	else if(LED_Num == 2){
		LED_Control(UART2_TXD_LED_GPIO_Port, UART2_TXD_LED_Pin , LED_Off);
	}
}

void Sub_Com_RX_LED_On(uint8_t LED_Num){
	if(LED_Num == 1){
		LED_Control(UART1_RXD_LED_GPIO_Port, UART1_RXD_LED_Pin , LED_On);
	}
	else if(LED_Num == 2){
		LED_Control(UART2_RXD_LED_GPIO_Port, UART2_RXD_LED_Pin , LED_On);
	}
}

void Sub_Com_RX_LED_Off(uint8_t LED_Num){
	if(LED_Num == 1){
		LED_Control(UART1_RXD_LED_GPIO_Port, UART1_RXD_LED_Pin , LED_Off);
	}
	else if(LED_Num == 2){
		LED_Control(UART2_RXD_LED_GPIO_Port, UART2_RXD_LED_Pin , LED_Off);
	}
}

void Sub_Com_RX_LED_Off_Start(uint8_t LED_Num){

	if(LED_Num == 1){
		HAL_TIM_Base_Start_IT(&htim6);
	}
	else if(LED_Num == 2){
		HAL_TIM_Base_Start_IT(&htim7);
	}


}
void Sub_Com_ST_LED_On(uint8_t LED_Num){
	if(LED_Num == 1){
		LED_Control(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin , LED_On);
	}
	else if(LED_Num == 2){
		LED_Control(ERR_UART2_LED_GPIO_Port, ERR_UART2_LED_Pin , LED_On);
	}
}

void Sub_Com_ST_LED_Off(uint8_t LED_Num){
	if(LED_Num == 1){
		LED_Control(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin , LED_Off);
	}
	else if(LED_Num == 2){
		LED_Control(ERR_UART2_LED_GPIO_Port, ERR_UART2_LED_Pin , LED_Off);
	}
}


void Sub_Com_TX_UART(uint8_t UART_Num,  const uint8_t *Uart_Tx_Data){
	if(UART_Num == 1){
		HAL_UART_Transmit(&huart4, Uart_Tx_Data, 8,100);
	}
	else if(UART_Num == 2){
		HAL_UART_Transmit(&huart2, Uart_Tx_Data, sizeof(Uart_Tx_Data),100);
	}
}


void Check_UART_Receive(int Port, int check_val){

	uint8_t Uart_crc = 0 ;
	uint8_t Uart_temp_data[32];
	uint8_t Repeater_Data_tmp[5];

	uint8_t Isolation_mode= 0, V24_Status= 0;

	uint8_t Ch1_Out_Short= 0;

//	Repeater_Data_tmp[0]
//	주소 정보

//	Repeater_Data_tmp[1]
//0x00 : 확인 안됨
//0xA0 : 중계기
//0x40 : 먼지
//0x44 : 온도

//	Repeater_Data_tmp[2]
//	Repeater(입력)
//	bit7 : ch4 단락, bit6 : ch3 단락, bit5 : ch2 단락, bit4 : ch1 단락,
//	bit3 : ch4 입력, bit2 : ch3 입력, bit1 : ch2 입력, bit0 : ch1 입력
//
//	analog
//	"먼지 : '0%' ~ '25.5%'
//	온도 : '0.0' ~ '255'"
//	"먼지 : 0x00 ~ 0xFF
//	온도 : 0x00 ~ 0xFF

//	Repeater_Data_tmp[3]
//   Repeater (출력)
//   bit7 : ch4 단선, bit6 : ch3 단선, bit5 : ch2 단선, bit4 : ch1 단선,
//   bit3 : ch4 출력, bit2 : ch3 출력, bit1 : ch2 출력, bit0 : ch1 출력
//
//   analog
//   bit7 : 0: 상위 통신 정상,1: 상위 통신 단락 이상,
//   bit6 : 0: 상위 통신 정상,1: 상위 통신 단선 이상,
//   bit5 : 0: 하위 통신 정상,1: 하위 통신 단락 이상,
//   bit4 : 0: 하위 통신 정상,1: 하위 통신 단선 이상,
//   bit3 :
//   0b???? ?1?? : 통신 LED 상태 off
//   0b???? ?0?? : 통신 LED 상태 on
//   0b???? ???1 : Fire LED 상태 on
//   0b???? ???0 : Fire LED 상태 off
//
//	Repeater_Data_tmp[4]
//   bit7 :
//   bit6 :
//   bit5 :
//   bit4 : 0: 축적 off, 1: 축적 on
//  bit3 : 0: 상위 통신 정상,1: 상위 통신 단락 이상,
//  bit2 : 0: 상위 통신 정상,1: 상위 통신 단선 이상,
//  bit1 : 0: 기본 기능, 1: Isolation 기능
//  bit0 : 0 정상 , 1 : 24V 입력 없음
//
//  analog
//  예비


	if(check_val == 1){

		for(int i=0; i<32 ; i++){
			Uart_temp_data[i] = 0;
		}

		int repeater_mode;
		if(Port == OUT_PORT){
			Sub_Com_RX_LED_On(1);
			for(int i=0; i< sizeof(Uart_temp_data) ; i++){
				Uart_temp_data[i] = UART_RX_buf[0][i];
			}
			REPEATER_Response_Regster[Port][Uart_temp_data[1]-1] = 1;
		}
		else if(Port == IN_PORT){
			Sub_Com_RX_LED_On(2);
			for(int i=0; i<sizeof(Uart_temp_data); i++){
				Uart_temp_data[i] = UART_RX_buf[1][i];
			}
			REPEATER_Response_Regster[Port][Uart_temp_data[1]-1] = 1;
		}
		Uart_crc = Uart_temp_data[1]^Uart_temp_data[2]^Uart_temp_data[3]^Uart_temp_data[4];
		if(Uart_crc  == Uart_temp_data[5]){

			//Repeater_Data_tmp[0]
			// Address
			Repeater_Data_tmp[0] = Uart_temp_data[1];

			//Repeater_Data_tmp[1]
			// 0x00 : 확인 안됨
			//0xA0 : 중계기
			//0x40 : 먼지
			//0x44 : 온도

			//UART_TX_buf[4] = ((Rep_V24_value&0x01)<<7)|((Ch1_Out_Short&0x01)<<6)|
			//				((REPEATOR_MODE&0x01)<<5)|((ISO_MODE&0x01)<<4)|
			//				(Rep_port_open[3]<<3)|(Rep_port_open[2]<<2)|
			//				(Rep_port_open[1]<<1)|(Rep_port_open[0]<<0);

			repeater_mode = 0;
			if((Uart_temp_data[4]>>4) == 0x4 ){	// Analog

				Repeater_Data_tmp[1] = Uart_temp_data[4];
				Uart_temp_data[3] = Anal_Process(Uart_temp_data[1], Uart_temp_data[4], Uart_temp_data[3]);
				repeater_mode = 0;
			}
			else if( ((Uart_temp_data[4]>>5)& 0x01) == 0x01 ){//Notmal Repeater
				Isolation_mode = ((Uart_temp_data[4]>>4)& 0x01);

				if(Isolation_mode == 0){
					repeater_mode = 1;
					Repeater_Data_tmp[1] = 0xA0;
				}
				else{
					repeater_mode = 1;
					Repeater_Data_tmp[1] = 0xA2;
				}
			}
			else{											//Open Short Repeater
				repeater_mode = 1;
				Repeater_Data_tmp[1] = 0xA1;
			}


			//REPEATER data
			//UART_TX_buf[2] = (Rep_port_Fuse_Open_Mode[3]<<7)|(Rep_port_Fuse_Open_Mode[2]<<6)|
			//				(Rep_port_Fuse_Open_Mode[1]<<5)|(Rep_port_Fuse_Open_Mode[0]<<4)|
			//				(Rep_Output_value[3]<<3)|(Rep_Output_value[2]<<2)|
			//				(Rep_Output_value[1]<<1)|(Rep_Output_value[0]<<0);
			//UART_TX_buf[3] = ((Rep_port_Charge_Mode[3] & 0x01)<<7)|((Rep_port_Charge_Mode[2] & 0x01)<<6)|
			//				((Rep_port_Charge_Mode[1] & 0x01)<<5)|((Rep_port_Charge_Mode[0] & 0x01)<<4)|
			//				(Rep_Input_value[3]<<3)|(Rep_Input_value[2]<<2)|
			//				(Rep_Input_value[1]<<1)|(Rep_Input_value[0]<<0);
			//
			//UART_TX_buf[4] = ((Rep_V24_value&0x01)<<7)|((Ch1_Out_Short&0x01)<<6)|
			//				((REPEATOR_MODE&0x01)<<5)|((ISO_MODE&0x01)<<4)|
			//				(Rep_port_open[3]<<3)|(Rep_port_open[2]<<2)|
			//				(Rep_port_open[1]<<1)|(Rep_port_open[0]<<0);

			//	Repeater_Data_tmp[2]
			//	Repeater(입력)
			//	bit7 : ch4 단락, bit6 : ch3 단락, bit5 : ch2 단락, bit4 : ch1 단락,
			//	bit3 : ch4 입력, bit2 : ch3 입력, bit1 : ch2 입력, bit0 : ch1 입력
			//
			//	analog
			//	"먼지 : '0%' ~ '25.5%'
			//	온도 : '0.0' ~ '255'"
			//	"먼지 : 0x00 ~ 0xFF
			//	온도 : 0x00 ~ 0xFF

			if(repeater_mode ==1){
				if(V24_Status == 1){	//24V failure
					Repeater_Data_tmp[2] = 0x00;
				}
				else{
					Repeater_Data_tmp[2] = ((Uart_temp_data[3]&0x0f))|((Uart_temp_data[4]&0x0f)<<4);

					if((Uart_temp_data[3]&0x0f) == 0){

					}
					else{
						if(REPEATER_first_fire == 0){
							REPEATER_first_fire = 1;
						}
						else if(REPEATER_first_fire == 1){
							REPEATER_first_fire = 0;
						}

					}
				}
			}
			else{
				Repeater_Data_tmp[2] = Uart_temp_data[3];
			}


			//	Repeater_Data_tmp[3]
			//Repeater (출력)
			//bit7 : ch4 단선, bit6 : ch3 단선, bit5 : ch2 단선, bit4 : ch1 단선,
			//bit3 : ch4 출력, bit2 : ch3 출력, bit1 : ch2 출력, bit0 : ch1 출력


			//analog
			//bit7 : 0: 상위 통신 정상,1: 상위 통신 단락 이상,
			//bit6 : 0: 상위 통신 정상,1: 상위 통신 단선 이상,
			//bit5 : 0: 하위 통신 정상,1: 하위 통신 단락 이상,
			//bit4 : 0: 하위 통신 정상,1: 하위 통신 단선 이상,
			//bit3 :
			//0b???? ?1?? : 통신 LED 상태 off
			//0b???? ?0?? : 통신 LED 상태 on
			//0b???? ???1 : Fire LED 상태 on
			//0b???? ???0 : Fire LED 상태 off

			if(repeater_mode ==1){
				Repeater_Data_tmp[3] = Uart_temp_data[2];
			}
			else{
				Repeater_Data_tmp[3] = Uart_temp_data[2];
			}

			//	Repeater_Data_tmp[4]
//			 bit7 : 0 : 정상 , 1 : 24V 입력 없음,
//			 bit6 : 0: ch1 출력 정상,1: ch1 출력 단락,
//			 bit5 : 0: 확인 안됨,1: 중계기,
//			 bit4 : 0: 일반모드,1: 아이솔레이션,
//			 bit3 : Ch4  0: 축적 off, 1: 축적 on,
//			 bit2 : Ch3  0: 축적 off, 1: 축적 on,
//			 bit1 : Ch2  0: 축적 off, 1: 축적 on,
//			 bit0 : Ch1  0: 축적 off, 1: 축적 on,


			if(repeater_mode ==1){

				if(((Uart_temp_data[4]>>7)& 0x01) == 1){
					V24_Status = 0;
				}
				else{
					V24_Status = 1;
				}

				if(V24_Status == 0){
					Ch1_Out_Short = ((Uart_temp_data[4]>>6)& 0x01);
				}
				else{
					Ch1_Out_Short = 0;
				}

				if(Repeater_Data_tmp[1] == 0xA1){
					if(Ch1_Out_Short == 0){
						Ch1_Out_Short = 1;
					}
					else{
						Ch1_Out_Short = 0;
					}
				}
				else if(Repeater_Data_tmp[1] == 0xA0){
					Ch1_Out_Short = 0;
				}
				else if(Repeater_Data_tmp[1] == 0xA2){
					Ch1_Out_Short = 0;
				}

				//Isolation_mode = ((Uart_temp_data[4]>>4)& 0x01);

				/*
				Repeater_Data_tmp[4] = (V24_Status<<7)|(Ch1_Out_Short<<6)|(repeater_mode<<5)|(Isolation_mode<<4)|
						((Uart_temp_data[3]>>4) & 0x0f);
						*/
				Repeater_Data_tmp[4] = (V24_Status<<7)|(Ch1_Out_Short<<6)|(repeater_mode<<5)|
										((Uart_temp_data[3]>>4) & 0x0f);
			}
			else{
				Isolation_mode = (Uart_temp_data[2] >> 2) & 0x01;
				Repeater_Data_tmp[4] = (Isolation_mode<<4);
			}


			for(int i=0 ; i<5 ; i++){
				Group_Data[6+(Repeater_Data_tmp[0]-1)*5 + i] = Repeater_Data_tmp[i];
			}

			Check_Repeater_Charge(Repeater_Data_tmp[0]);

		}
		else{

		}
		//Sub_Com_RX_LED_Off(1);
		//Sub_Com_RX_LED_Off(2);
		Sub_Com_RX_LED_Off_Start(1);
		Sub_Com_RX_LED_Off_Start(2);
	}
	else{
		if(Port == OUT_PORT){
			if(UART_TX_buf[0][1] != 0){
				for(int i=0 ; i<4 ; i++){
					Group_Data[6+(UART_TX_buf[0][1]-1)*5 + i + 1] = 0;
				}
				for(int i=0 ; i<7 ; i++){
					UART_RX_buf[0][i] = 0;
				}
			}
		}
		else if(Port == IN_PORT){
			if(UART_TX_buf[1][1] != 0){
				for(int i=0 ; i<4 ; i++){
					Group_Data[6+(UART_TX_buf[1][1]-1)*5 + i +1] = 0;
				}
				for(int i=0 ; i<7 ; i++){
					UART_RX_buf[1][i] = 0;
				}
			}
			//REPEATER_Response_Regster[Port][UART_TX_buf[1][1]-1] = 1;
		}
	}

	for(int i=0; i< UART_buf_len; i++){
		UART_RX_buf[0][i] = 0;
		UART_RX_buf[1][i] = 0;
		UART_RX_buf_tmp[0][i] = 0;
		UART_RX_buf_tmp[1][i] = 0;
	}
	check_val = 0;
}

void Check_Repeater_Charge(uint8_t num){

	uint8_t charge_tmp, save_charge_tmp;
	uint8_t Command;

	charge_tmp = Group_Data[6+(num-1)*5 + 4] & 0x0f;
	save_charge_tmp = (REPEATER_Acc_Set_Data[num-1]) & 0xf;

	if( charge_tmp != save_charge_tmp){

		if(save_charge_tmp == 0){
			Command = 0xC4;
		}
		else{
			Command = 0xC2;
		}

		if(REPEATER_Possible[num-1] == 0){
			Sub_Send_Command(OUT_PORT, num, Command, REPEATER_Acc_Set_Data[num-1]);
			HAL_Delay(10);
		}
		else if(REPEATER_Possible[num-1] == 1){
			Sub_Send_Command(IN_PORT, num, Command, REPEATER_Acc_Set_Data[num-1]);
			HAL_Delay(10);
		}

	}
}

void Set_Acc_OnOff(void){

	uint8_t Command , data , Device, Acc_Time, Acc_Ch;
	uint8_t Acc_Ch_Regster[4];

	Command = 0xC2;

	for(int i =0; i< Repeater_Number ; i++){

		Device = REPEATER_Regster[i] & 0x0f;

		if((Device == 1)|(Device == 2)|(Device == 3)){

			REPEATER_Charge_Regster[0] = (~(REPEATER_Regster[i]&0xf0)>>4)&0x01;
			REPEATER_Charge_Regster[1] = (~(REPEATER_Regster[i]&0xf0)>>5)&0x01;
			REPEATER_Charge_Regster[2] = (~(REPEATER_Regster[i]&0xf0)>>6)&0x01;
			REPEATER_Charge_Regster[3] = (~(REPEATER_Regster[i]&0xf0)>>7)&0x01;

			Acc_Time = (REPEATER_Acc_Set_Data[i] >> 4 ) & 0x0f;

			Acc_Ch = (REPEATER_Acc_Set_Data[i] >> 0 ) & 0x0f;

			Acc_Ch_Regster[3] = (Acc_Ch >> 3 ) & 0x01;
			Acc_Ch_Regster[2] = (Acc_Ch >> 2 ) & 0x01;
			Acc_Ch_Regster[1] = (Acc_Ch >> 1 ) & 0x01;
			Acc_Ch_Regster[0] = (Acc_Ch >> 0 ) & 0x01;

			Acc_Ch_Regster[3] = Acc_Ch_Regster[3] & REPEATER_Charge_Regster[3];
			Acc_Ch_Regster[2] = Acc_Ch_Regster[2] & REPEATER_Charge_Regster[2];
			Acc_Ch_Regster[1] = Acc_Ch_Regster[1] & REPEATER_Charge_Regster[1];
			Acc_Ch_Regster[0] = Acc_Ch_Regster[0] & REPEATER_Charge_Regster[0];

			Acc_Ch = (Acc_Ch_Regster[3]<<3)|(Acc_Ch_Regster[2]<<2)|(Acc_Ch_Regster[1]<<1)|(Acc_Ch_Regster[0]<<0);

			data = ((Acc_Time<<4)&0xf0)|((Acc_Ch<<0)&0x0f);

			//data = REPEATER_Acc_Set_Data[i];

			REPEATER_Acc_Com_Data[i] = data;
			if(REPEATER_Possible[i] == 0){
				Sub_Send_Command(OUT_PORT, i+1, Command, data);
				HAL_Delay(10);
			}
			else if(REPEATER_Possible[i] == 1){
				Sub_Send_Command(IN_PORT, i+1, Command, data);
				HAL_Delay(10);
			}
		}
	}
}
