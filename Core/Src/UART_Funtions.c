
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UART_Funtions.c
  * @brief          : UART_Funtions
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

#include "main.h"
#include "UART_Funtions.h"
#include "Sub_Com.h"
#include "W25Q_Load_Data.h"
#include "w25qxx.h"
#include "w25qxxConf.h"
#include "ReadWriteFlash.h"
#include "Internal_Flash.h"

unsigned short CRC16(unsigned char *crcdata, unsigned short usDataLen){

	unsigned short crc = 0xffff;

	for(int i=0;i<usDataLen ; i++){
		crc = crc^((*crcdata) << 8);
		crcdata++;
		for(int j=0;j<8;j++){
			if(crc & 0x8000){
				crc = (crc << 1) ^ 0x1021;
			}
			else{
				crc = crc << 1;
			}
		}
	}

	return crc;
}



//void UI_Receiver(void)
//{
//	uint8_t Uart2_crc = 0 ;
//	HAL_GPIO_WritePin(UI_RX_LED_GPIO_Port, UI_RX_LED_Pin, GPIO_PIN_RESET);
//
//	// Start saving data
//	if((UI_UART_buf_tmp == 0x53)&( UI_UART_State == 0)){
//		UI_UART_State = 1;
//		UI_UART_buf_count = 0;
//	}
//
//	// Save data from when data storage begins
//	if(UI_UART_State == 1){
//		UI_UART_buf[UI_UART_buf_count] = UI_UART_buf_tmp;
//	    UI_UART_buf_count++;
//	}
//
//	// Check the start bit of the saved data.
//	if(UI_UART_buf_count > 3){
//		if((UI_UART_buf[0] == 0x53) &(UI_UART_buf[1] == 0x54)){
//		}
//	    else{
//	      UI_UART_State = 0;
//	      UI_UART_buf_count = 0;
//	    }
//	  }
//
//	// If the end bit is confirmed, data reception is completed after checking the crc.
//	if((UI_UART_buf[UI_UART_buf_count - 1] == 0x44) & (UI_UART_buf[UI_UART_buf_count - 2] == 0x45)){
//
//		for(int i = 0; i <UI_UART_buf_count - 5 ; i++){
//		    Uart2_crc = Uart2_crc ^ UI_UART_buf[i+2];
//		}
//
//		if(Uart2_crc == UI_UART_buf[UI_UART_buf_count-3]){
//			UI_UART_Receive_complete = 1;
//			UI_UART_buf_count_tmp = UI_UART_buf_count;
//			UI_UART_State = 0;
//			UI_UART_buf_count = 0;
//		}
//		else{
//			UI_UART_Receive_complete = 0;
//			UI_UART_buf_count_tmp = 0;
//			UI_UART_State = 0;
//			UI_UART_buf_count = 0;
//			for(int i = 0; i < UI_UART_buf_len ; i++){
//				UI_UART_buf[i] = 0;
//			}
//		}
//
//	  }
//	HAL_GPIO_WritePin(UI_RX_LED_GPIO_Port, UI_RX_LED_Pin, GPIO_PIN_SET);
//
//	//HAL_UART_Receive_IT(&huart1, &UI_UART_buf_tmp, 1);
//	//HAL_GPIO_WritePin(UI_DC_GPIO_Port, UI_DC_Pin, GPIO_PIN_RESET); // rx mode
//
//}

void UI_Com_Func(void){

	if(UI_UART_RX_buf[5] == CCU_Address){

		if(CCU_Mode == 1){

			if(UI_UART_RX_buf[4] == 1){
				Run_UI_Com();
			}
		}
		else{
			Run_UI_Com();
		}
	}
	else if((CCU_Address == 1)&(UI_UART_RX_buf[4] == 4)&(CCU_Mode == 1)){
		if(UI_UART_RX_buf[3] == 0x50){	//'P' Request for relay board power board information
			UI_Cmd_Func_P();
		}

	}
}

void Run_UI_Com(void){

	 if(UI_UART_RX_buf[3] == 0x41){	// 'A' Repeater information request: Read all information of the requested system.
		 UI_Com_Func_A();
	 }
	 else if(UI_UART_RX_buf[3] == 0x53){	//'S' Repeater settings: Repeater output settings
		 //UI_Com_Func_S();
		 UI_Com_Func_S_Re(1);
		 Sub_Set_Mode = 1;

	 }
	 else if(UI_UART_RX_buf[3] == 0x43){	//'C' Delete repeater information

	 }
	 else if(UI_UART_RX_buf[3] == 0x49){	//'I' Register repeater information
		 UI_Com_Func_Infor_set();
	 }
	 else if(UI_UART_RX_buf[3] == 0x52){	//'R' Repeater recovery request
		 UI_Com_Func_Reset();
	 }
	 else if(UI_UART_RX_buf[3] == 0x47){	//'G' Repeater accumulation setting
		 UI_Com_Func_Acc_OnOff();
		 Sub_Set_Mode = 1;
	 }
	 else if(UI_UART_RX_buf[3] == 0x44){	//'D' Registering repeater accumulated information
		 UI_Com_Func_Acc_Set();
		 //UI_Com_Func_Acc_Set_Re(2);
		 Sub_Set_Mode = 1;
	 }
	 else if(UI_UART_RX_buf[3] == 0x50){	//'P' Request for relay board power board information

	 }
	 else if(UI_UART_RX_buf[3] == 0x54){	//'T' Repeater power board battery test command

	 }
	 else if(UI_UART_RX_buf[3] == 0x56){	//'V' Version Info command
		 UI_Com_Func_V();
	 }
	 else if(UI_UART_RX_buf[3] == 0x6C){	//'R' Repeater recovery request
		 UI_Com_Func_Anal_LED_Set(1);
	 }
}
// 'A' Repeater information request: Read all information of the requested system.
void UI_Com_Func_A(void){

	uint8_t Crc_Data_in = 0;
	uint8_t Crc_Data[Repeater_Data_Total - 6];
	short cal_crc16;
	uint16_t Send_Data_Len = Repeater_Data_Total;

	for(int i=2 ; i < UI_UART_buf_count_Save - 3 ;i++){
		Crc_Data_in = Crc_Data_in ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data_in == UI_UART_RX_buf[6]){
		for(int i=0; i< Repeater_Data_Total ; i++){
			Rep_Group_Data[i] = Group_Data[i];
		}

		Rep_Group_Data[0] = 83;
		Rep_Group_Data[1] = 84;
		Rep_Group_Data[2] = 106;
		Rep_Group_Data[3] = 114;
		/*
		Rep_Group_Data[4] = 250;
		Rep_Group_Data[5] = UI_UART_RX_buf[5];
		Rep_Group_Data[6] = UI_UART_RX_buf[6];
		*/
		Rep_Group_Data[4] = UI_UART_RX_buf[4];
		Rep_Group_Data[5] = UI_UART_RX_buf[5];

		for(int i=0; i< (Repeater_Data_Total - 6 ); i++){
			Crc_Data[i] = Rep_Group_Data[i+2];
		}

		cal_crc16 = CRC16(Crc_Data, sizeof(Crc_Data)/sizeof(uint8_t) );

		Rep_Group_Data[Repeater_Data_Total- 4] = (cal_crc16 >> 8)& 0xff;
		Rep_Group_Data[Repeater_Data_Total- 3] = (cal_crc16 >> 0)& 0xff;

	//	LED_Control(UI_TX_LED_GPIO_Port, UI_TX_LED_Pin , LED_On);
	//	LED_Control(SYS_LED1_GPIO_Port, SYS_LED1_Pin , LED_On);
	//	LED_Control(UART1_TXD_LED_GPIO_Port, UART1_TXD_LED_Pin , LED_On);
		//HAL_Delay(1);
	//	HAL_GPIO_WritePin(UI_DC_GPIO_Port, UI_DC_Pin, GPIO_PIN_SET); // sp3485EN-L is tx mode
		//Rs485 Tx mode//
		LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_On);
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

		//HAL_UART_Transmit_DMA(&hlpuart1, Rep_Group_Data, Send_Data_Len);
		//HAL_UART_Receive_IT(&hlpuart1, Rep_Group_Data, 1260);
		//HAL_Delay(10);

		HAL_UART_Transmit(&hlpuart1, Rep_Group_Data, Send_Data_Len,1000);
		HAL_Delay(15);
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

		LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_Off);
	}
	Reset_UI_uart = 0;

	///////////If the DMA UART is malfunctioning, change the reception mode to an interrupt./////


}

// 'S' Repeater OutPut Setting.
void UI_Com_Func_S(void){

//	Sub_Send_Command_Set(uint8_t Send_Port, uint8_t Address, uint8_t Output)

	uint8_t Crc_Data = 0, End_Num = 11 ;

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data == UI_UART_RX_buf[End_Num]){
		uint8_t Address= UI_UART_RX_buf[6] , Command = 0xC1;

		/*
		if(Com_Out_Status == Com_On){
			if(REPEATER_Possible[Address-1] == 0){
				Sub_Send_Command(OUT_PORT, Address, Command, UI_UART_RX_buf[8]);
			}
		}
		if(Com_In_Status == Com_On){
			if(REPEATER_Possible[Address] == 1){
			  Sub_Send_Command(IN_PORT, Address, Command, UI_UART_RX_buf[8]);
			}
		}
		*/
		if(REPEATER_Possible[Address-1] == 0){
			Sub_Send_Command(OUT_PORT, Address, Command, UI_UART_RX_buf[8]);
			HAL_Delay(10);
			Read_Reapeter_Set_Data(OUT_PORT, Address);
		}
		else if(REPEATER_Possible[Address-1] == 1){
			Sub_Send_Command(IN_PORT, Address, Command, UI_UART_RX_buf[8]);
			HAL_Delay(10);
			Read_Reapeter_Set_Data(IN_PORT, Address);
		}
	}
}

void UI_Com_Func_S_Re(uint8_t Num){

	uint8_t Crc_Data = 0, End_Num = 11 ;
	uint8_t Set_Data = 0, Load_Data = 0 ;

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data == UI_UART_RX_buf[End_Num]){

		uint8_t Address= UI_UART_RX_buf[6] , Command = 0xC1;

		for(int j=0; j< Num; j++){

			if(REPEATER_Possible[Address-1] == 0){
				Sub_Send_Command(OUT_PORT, Address, Command, UI_UART_RX_buf[8]);
				HAL_Delay(20);
				Sub_Send_Command(OUT_PORT, Address, Command, UI_UART_RX_buf[8]);
				HAL_Delay(20);
				Read_Reapeter_Set_Data(OUT_PORT, Address);
			}
			else if(REPEATER_Possible[Address-1] == 1){
				Sub_Send_Command(IN_PORT, Address, Command, UI_UART_RX_buf[8]);
				HAL_Delay(20);
				Sub_Send_Command(IN_PORT, Address, Command, UI_UART_RX_buf[8]);
				HAL_Delay(20);
				Read_Reapeter_Set_Data(IN_PORT, Address);
			}
			Set_Data = UI_UART_RX_buf[8] & 0x0F;
			Load_Data = Group_Data[6+5*(Address-1) + 2] & 0x0F;
			if(Set_Data == Load_Data){
				break;
			}
			else{
				HAL_Delay(1);
			}
		}
	}

}

void UI_Com_Func_Reset(void){

	uint8_t Crc_Data = 0, End_Num = 6 ;

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data == UI_UART_RX_buf[End_Num]){
		NVIC_SystemReset(); // 시스템 초기화를 합니다
	}

}
/*
void UI_Com_Func_Acc_OnOff(void){

	uint8_t Crc_Data = 0, End_Num = 7 ;
	uint8_t Read_Charge_val ;
	uint8_t Command , data , Acc_Time;

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}


	if(Crc_Data == UI_UART_RX_buf[End_Num]){

		if(UI_UART_RX_buf[6] == 0){  //off
			Command = 0xC4;
			//data = 0;
		}
		else{		//on
			Command = 0xC2;
			Acc_Time = (UI_UART_RX_buf[6]<<4) & 0xf0;
			//data = (UI_UART_RX_buf[6]<<4 & 0xf0) | REPEATER_Acc_Set_Data[i];
		}

		for(int i =0; i< Repeater_Number ; i++){
			if((REPEATER_Regster[i] == 1)|(REPEATER_Regster[i] == 2)){

				if(Command == 0xC4){
					data = 0;
				}
				else if(Command == 0xC2){
					data = Acc_Time| REPEATER_Acc_Set_Data[i];
				}

				if(REPEATER_Possible[i] == 0){
					Sub_Send_Command(OUT_PORT, i+1, Command, data);
					HAL_Delay(10);
					//Read_Reapeter_Single_Data(i+1, OUT_PORT);
				}
				else if(REPEATER_Possible[i] == 1){
					Sub_Send_Command(IN_PORT, i+1, Command, data);
					HAL_Delay(10);
					//Read_Reapeter_Single_Data(i+1, IN_PORT);
				}
				Read_Charge_val = Group_Data[6+(i)*5 + 4];
				Read_Charge_val = (Read_Charge_val >> 4) & 0x01;


				if((Command == 0xC4)&(Read_Charge_val == 0)){

				}
				else{
					if(REPEATER_Possible[i] == 0){
						Sub_Send_Command(OUT_PORT, i+1, Command, data);
						HAL_Delay(10);
					}
					else if(REPEATER_Possible[i] == 1){
						Sub_Send_Command(IN_PORT, i+1, Command, data);
						HAL_Delay(10);
					}
				}

				if((Command == 0xC2)&(Read_Charge_val == 1)){

				}
				else{
					if(REPEATER_Possible[i] == 0){
						Sub_Send_Command(OUT_PORT, i, Command, data);
						HAL_Delay(10);
					}
					else if(REPEATER_Possible[i] == 1){
						Sub_Send_Command(IN_PORT, i, Command, data);
						HAL_Delay(10);
					}
				}


			}
		}
		Crc_Data = 0;
	}


}
*/

void UI_Com_Func_Acc_OnOff(void){

	uint8_t Crc_Data = 0, End_Num = 7 ;
	uint8_t Command , data;

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data == UI_UART_RX_buf[End_Num]){

		if(UI_UART_RX_buf[6] == 0){  //off
			Command = 0xC4;
			//data = 0;
			for(int i=0; i<Repeater_Number; i++){
				REPEATER_Acc_Set_Data[i] = 0;
			}
//			Write_W25Q_Device_Info();

		}
		else{		//on
			Command = 0xC2;
			/*
			Acc_Time = (UI_UART_RX_buf[6]<<4) & 0xf0;
			data = (UI_UART_RX_buf[6]<<4 & 0xf0) | 0x0f;
			*/
			data = UI_UART_RX_buf[6];

			for(int i=0; i<Repeater_Number; i++){
				REPEATER_Acc_Set_Data[i] = (data<<4 | 0x0f);
			}
			//Write_W25Q_Device_Info();
//			Write_To_Flash(Charge_In_ADDR_FLASH, REPEATER_Acc_Set_Data ,Repeater_Number);
		}

		for(int j = 0; j<3; j++){
			for(int i =0; i< Repeater_Number ; i++){

				data = REPEATER_Acc_Set_Data[i];

				if((REPEATER_Regster[i] == 1)|(REPEATER_Regster[i] == 2)|(REPEATER_Regster[i] == 3)){

					if(Command == 0xC4){
						data = 0;
					}
					else if(Command == 0xC2){
						data = REPEATER_Acc_Set_Data[i];
					}
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

	}

}

void UI_Com_Func_Acc_Set(void){

	uint8_t Crc_Data = 0, End_Num = UI_UART_buf_count_Save - 3 ;
	uint8_t Command , data;

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data == UI_UART_RX_buf[End_Num]){

		Command = 0xC2;
		data = UI_UART_RX_buf[7];
		if(REPEATER_Possible[UI_UART_RX_buf[6]-1] == 0){
			Sub_Send_Command(OUT_PORT, UI_UART_RX_buf[6], Command, data);
			HAL_Delay(20);
		}
		else if(REPEATER_Possible[UI_UART_RX_buf[6]-1] == 1){
			Sub_Send_Command(IN_PORT, UI_UART_RX_buf[6], Command, data);
			HAL_Delay(20);
		}
		REPEATER_Acc_Set_Data[UI_UART_RX_buf[6]-1] = data;
		//Write_W25Q_Device_Info();
		//Write_To_Flash(Charge_In_ADDR_FLASH, REPEATER_Acc_Set_Data ,Repeater_Number);
	}

	uint8_t Address= UI_UART_RX_buf[6];

	if(REPEATER_Possible[Address-1] == 0){
		Read_Reapeter_Set_Data(OUT_PORT, Address);
	}
	else if(REPEATER_Possible[Address-1] == 1){
		Read_Reapeter_Set_Data(IN_PORT, Address);
	}

}

void UI_Com_Func_Acc_Set_Re(uint8_t Num){

	uint8_t Crc_Data = 0, End_Num = UI_UART_buf_count_Save - 3 ;
	uint8_t Command , data;
	uint8_t Set_Data = 0, Load_Data = 0 ;
	uint8_t Address= UI_UART_RX_buf[6];

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data == UI_UART_RX_buf[End_Num]){

		Command = 0xC2;
		data = UI_UART_RX_buf[7];

		for(int j=0; j< Num;j++){
			if(REPEATER_Possible[Address-1] == 0){
				Sub_Send_Command(OUT_PORT, Address, Command, data);
				HAL_Delay(20);
				Sub_Send_Command(OUT_PORT, Address, Command, data);
				HAL_Delay(20);
				Read_Reapeter_Set_Data(OUT_PORT, Address);
			}
			else if(REPEATER_Possible[Address-1] == 1){
				Sub_Send_Command(IN_PORT, Address, Command, data);
				HAL_Delay(20);
				Sub_Send_Command(IN_PORT, Address, Command, data);
				HAL_Delay(20);
				Read_Reapeter_Set_Data(IN_PORT, Address);
			}
			Set_Data = UI_UART_RX_buf[7] & 0x0F;
			Load_Data = Group_Data[6+5*(Address-1) + 3] & 0x0F;
			if(Set_Data == Load_Data){
				break;
			}
			else{
				HAL_Delay(1);
			}

		}

		REPEATER_Acc_Set_Data[UI_UART_RX_buf[6]-1] = data;
		//Write_W25Q_Device_Info();
		//Write_To_Flash(Charge_In_ADDR_FLASH, REPEATER_Acc_Set_Data ,Repeater_Number);
	}

}
void Check_UI_UART_Receive(int check_val){

	//check_val = 0;

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);

//	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
//	/* Enable the UART Data Register not empty Interrupt */
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
//
//	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//	__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
//	/* Enable the UART Data Register not empty Interrupt */
//	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);

	Sub_Set_Mode = 0;
	if( check_val == 1){
	  UI_Com_Func();
	  UI_UART_Receive_complete = 0;
	  UI_UART_buf_count_tmp = 0;
	  for(int i = 0; i < UI_UART_buf_len ; i++){
		  UI_UART_RX_buf[i] = 0;
	  }


	  if(Mini_Mode == 1){

		  while(Sub_Set_Mode == 1){
			  Sub_Set_Mode = 0;
			  for(int i = 0; i<100; i++){
				  if( check_val == 1){
					  UI_Com_Func();
					  UI_UART_Receive_complete = 0;
					  UI_UART_buf_count_tmp = 0;
					  for(int i = 0; i < UI_UART_buf_len ; i++){
						  UI_UART_RX_buf[i] = 0;
					  }
					  break;
				  }
				  HAL_Delay(1);
			  }
		  }

		  /*
		  if(Sub_Set_Mode == 1){

		  }
		  */
	  }

	}
}

// receiver
void UI_Cmd_Func_P(void){
	uint8_t num = 0, Rse_Data = 0;
	if(UI_UART_RX_buf[4] % 8 == 0)
	{
		num = 7;
	}
	else
	{
		num = (UI_UART_RX_buf[4] % 8) - 1;
	}

	if( (CCU_Address % 2) == 1){
		if( (num >= 0) & (num < 4)){
			Rse_Data = 1;
		}
		else{
			Rse_Data = 0;
		}
	}
	else{
		if( (num >= 0) & (num < 4)){
			Rse_Data = 0;
		}
		else{
			Rse_Data = 1;
		}
	}

	if(Rse_Data == 1){
		for(int i=0; i< Pow_Bod_Data_Total; i++){
			Rep_Pow_Bod_Data[i] = Pow_Bod_Data[num][i];
		}

		uint8_t Crc_Val = 0;
		Rep_Pow_Bod_Data[0] = 0x53;
		Rep_Pow_Bod_Data[1] = 0x54;
		if(Rep_Pow_Bod_Data[2] == 0){
			Rep_Pow_Bod_Data[2] = 0x63;
		}
		Rep_Pow_Bod_Data[3] = 0x72;
		Rep_Pow_Bod_Data[4] = UI_UART_RX_buf[4];
		Rep_Pow_Bod_Data[5] = UI_UART_RX_buf[5];


		Rep_Pow_Bod_Data[Pow_Bod_Data_Total-2] = 0x45;
		Rep_Pow_Bod_Data[Pow_Bod_Data_Total-1] = 0x44;


		for(int i = 0 ; i < (Pow_Bod_Data_Total - 5) ; i++){
			Crc_Val = Crc_Val^Rep_Pow_Bod_Data[i+2];
		}
		Rep_Pow_Bod_Data[Pow_Bod_Data_Total-3] = Crc_Val;

		LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_On);
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

		HAL_UART_Transmit(&hlpuart1, Rep_Pow_Bod_Data, Pow_Bod_Data_Total,1000);
		HAL_Delay(15);
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

		LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_Off);

	}
//
//	HAL_Delay(1);
}

void UI_Com_Func_Infor_set(void)
{
	uint16_t cal_crc16, Uart_crc16_o;
	uint8_t Crc_Data_D[456 - 6];
	//uint8_t Save_Num = 1;

	for(int i=0; i< (UI_UART_buf_count_Save - 6); i++){
		Crc_Data_D[i] = UI_UART_RX_buf[i+2];
	}
	cal_crc16 = CRC16(Crc_Data_D, sizeof(Crc_Data_D)/sizeof(uint8_t));
	Uart_crc16_o = (UI_UART_RX_buf[UI_UART_buf_count_Save - 4]<<8)|(UI_UART_RX_buf[UI_UART_buf_count_Save - 3]);

	if(cal_crc16 == Uart_crc16_o){
		/*
		if(Write_Regi_data(Save_Num, UI_UART_RX_buf,UI_UART_buf_count_Save) == 1){
			for(int i=0; i<220; i++){
				REPEATER_Regster[i] = UI_UART_RX_buf[(i*2)+13];
			}
		}
		*/
		if(Write_To_Flash(Info_In_ADDR_FLASH, UI_UART_RX_buf,UI_UART_buf_count_Save) == 1){
			for(int i=0; i<220; i++){
				REPEATER_Regster[i] = UI_UART_RX_buf[(i*2)+13];
			}
		}

	}
}

void UI_Com_Func_V(void){


	uint8_t Crc_Data_in = 0;
	uint8_t Com_Crc_Data[Rep_Version_Data_Number - 6];
	short cal_crc16;

	for(int i=2 ; i < UI_UART_buf_count_Save - 3 ;i++){
		Crc_Data_in = Crc_Data_in ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data_in == UI_UART_RX_buf[6]){

		Rep_Version_Data[0] = 0x53;
		Rep_Version_Data[1] = 0x54;
		Rep_Version_Data[2] = 0x6A;
		Rep_Version_Data[3] = 0x76;

		Rep_Version_Data[4] = UI_UART_RX_buf[4];
		Rep_Version_Data[5] = UI_UART_RX_buf[5];

		Rep_Version_Data[Rep_Version_Data_Number - 2] = 0x45;
		Rep_Version_Data[Rep_Version_Data_Number - 1] = 0x44;

		for(int i=0; i<(Repeater_Number + 1); i++){
			Rep_Version_Data[6 + i*7] = i;
			for(int j = 0; j<Version_Cnt ; j++){
				Rep_Version_Data[6 + i*7 + 1 + j] = REPEATER_Version[i][j];
			}
		}

		for(int i=0; i< (Rep_Version_Data_Number - 6 ); i++){
			Com_Crc_Data[i] = Rep_Version_Data[i+2];
		}

		cal_crc16 = CRC16(Com_Crc_Data, sizeof(Com_Crc_Data)/sizeof(uint8_t) );

		Rep_Version_Data[Rep_Version_Data_Number- 4] = (cal_crc16 >> 8)& 0xff;
		Rep_Version_Data[Rep_Version_Data_Number- 3] = (cal_crc16 >> 0)& 0xff;

		//Rs485 Tx mode//
		LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_On);
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

		HAL_UART_Transmit(&hlpuart1, Rep_Version_Data, Rep_Version_Data_Number,1000);
		HAL_Delay(15);
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

		LED_Control(RUN_LED1_GPIO_Port, RUN_LED1_Pin , LED_Off);
	}
	Reset_UI_uart = 0;

	Sub_Version_Read_Cnt = 1;

}


void UI_Com_Func_Anal_LED_Set(uint8_t Num){

	uint8_t Crc_Data = 0, End_Num = 11 ;
	uint8_t Set_Data = 0, Load_Data = 0 ;
	uint8_t Command = 0xC1;
	uint8_t Mode = UI_UART_RX_buf[6], Address = 0;

	for(int i=2 ; i < End_Num ;i++){
		Crc_Data = Crc_Data ^ UI_UART_RX_buf[i];
	}

	if(Crc_Data == UI_UART_RX_buf[End_Num]){

		for(int j=0; j< Num; j++){

			for(int i=0; i< Repeater_Number ; i++){

				Address = i+1;

				if((REPEATER_Regster[i] > 3) &(REPEATER_Regster[i] < 9)){
					if(Mode == 0){
						Set_Data = Group_Data[6+5*(Address-1) + 2] & 0xF7;
					}
					else if(Mode == 1){
						Set_Data = Group_Data[6+5*(Address-1) + 2] | 0x08;
					}

					if(REPEATER_Possible[Address-1] == 0){
						Sub_Send_Command(OUT_PORT,Address , Command, Set_Data);
						HAL_Delay(10);
						Read_Reapeter_Set_Data(OUT_PORT, Address);
					}
					else if(REPEATER_Possible[Address-1] == 1){
						Sub_Send_Command(IN_PORT, Address, Command, Set_Data);
						HAL_Delay(10);
						Read_Reapeter_Set_Data(IN_PORT, Address);
					}
					Load_Data = Group_Data[6+5*(Address-1) + 2];
					if(Set_Data == Load_Data){
						break;
					}
					else{

						if(REPEATER_Possible[Address-1] == 0){
							Sub_Send_Command(OUT_PORT,Address , Command, Set_Data);
							HAL_Delay(10);
						}
						else if(REPEATER_Possible[Address-1] == 1){
							Sub_Send_Command(IN_PORT, Address, Command, Set_Data);
							HAL_Delay(10);
						}

					}

				}

			}
		}

	}

}
