/*
 * W25Q_Load_Data.c
 *
 *  Created on: Aug 18, 2025
 *      Author: root
 */


#include "main.h"
#include "w25qxx.h"
#include "w25qxxConf.h"
#include "W25Q_Load_Data.h"

#include "Compile_Data.h"
  /*
  W25qxx_ReadBytes(W25Q_Page_Read, 0, 256);

  W25qxx_ReadBytes(W25Q_Page_Read, 0, 256);
  for(int i=0; i< 256 ; i++){
	  W25Q_Page_Write[i] = i;
    }

  W25qxx_EraseBlock(0);
  W25qxx_ReadBytes(W25Q_Page_Read, 0, 256);
  W25qxx_WritePage(W25Q_Page_Write	,0 ,0,256);


  W25qxx_ReadBytes(W25Q_Page_Read, 0, 256);
*/

uint8_t W25Q_Page_Read[512];
uint8_t W25Q_Page_Write[512];

uint8_t F_Version_Year_Month;
uint8_t F_Version_Year_Month_Tmp;
uint8_t F_Version_Day_Tmp;
uint8_t F_Version_Hour_Tmp;

void Read_W25Q_Device_Info(void){

	uint8_t Temp_Sector[4096];

	W25qxx_ReadPage(W25Q_Page_Read, Infomation_Data_Address,0, Page_Size);

	F_Version_Year_Month_Tmp = W25Q_Page_Read[0];
	F_Version_Day_Tmp = W25Q_Page_Read[1];
	F_Version_Hour_Tmp = W25Q_Page_Read[2];

	W25qxx_ReadSector(Temp_Sector, REPEATER_Regster_Address, 0, 4096);

	for(int i=0; i<220; i++){
		REPEATER_Regster[i] = Temp_Sector[(i*2)+13];
	}

	W25qxx_ReadSector(Temp_Sector, Repeater_Acc_Set_Address, 0, 4096);

	for(int i=0; i<Repeater_Number; i++){
		REPEATER_Acc_Set_Data[i] = Temp_Sector[i];
	}

	if((F_Version_Year_Month_Tmp != F_Version_Year_Month)
			| (F_Version_Day_Tmp != F_Version_Day)
			| (F_Version_Hour_Tmp != F_Version_Hour) ){
		Write_W25Q_Device_Info();
	}

	if(REPEATER_Acc_Set_Data[0] == 0xff){
		for(int i=0; i<Repeater_Number; i++){
			REPEATER_Acc_Set_Data[i] = 0;
		}
		Write_W25Q_Device_Info();
	}
}

void Write_W25Q_Device_Info(void){

	W25qxx_EraseBlock(Info_Data_Block);

	for(int i=0; i<Page_Size; i++){
		W25Q_Page_Write[i] = 0;
	}

	W25Q_Page_Write[0] = F_Version_Year_Month;
	W25Q_Page_Write[1] = F_Version_Day;
	W25Q_Page_Write[2] = F_Version_Hour;

	W25Q_Page_Write[3] = Link_Table_Year_Month;
	W25Q_Page_Write[4] = Link_Table__Day;
	W25Q_Page_Write[5] = Link_Table_Hour;

	for(int i=0; i<7; i++){
		W25Q_Page_Write[6+i] = Serial_Num[i];
	}

	W25qxx_WritePage(W25Q_Page_Write, Infomation_Data_Address ,0,Page_Size);

	for(int i=0; i<Page_Size; i++){
		W25Q_Page_Read[i] = 0;
	}
	W25qxx_ReadPage(W25Q_Page_Read, Infomation_Data_Address, 0, Page_Size);

	F_Version_Year_Month = W25Q_Page_Read[0];
	F_Version_Day = W25Q_Page_Read[1];
	F_Version_Hour = W25Q_Page_Read[2];

	Link_Table_Year_Month = W25Q_Page_Read[3];
	Link_Table__Day = W25Q_Page_Read[4];
	Link_Table_Hour = W25Q_Page_Read[5];

	for(int i=0; i<7; i++){
		Serial_Num[i] = W25Q_Page_Read[6+i];
	}

	/////////////////////////////////////////////////////////////////////

	for(int i=0; i<Page_Size; i++){
		if(i< Repeater_Number){
			W25Q_Page_Write[i] = REPEATER_Regster[i];
		}
		else{
			W25Q_Page_Write[i] = 0;
		}
	}

	W25qxx_WritePage(W25Q_Page_Write, REPEATER_Regster_Address ,0,Page_Size);
	for(int i=0; i<Page_Size; i++){
		W25Q_Page_Read[i] = 0;
	}
	W25qxx_ReadPage(W25Q_Page_Read, REPEATER_Regster_Address, 0, Page_Size);

	for(int i=0; i<Repeater_Number; i++){
		REPEATER_Regster[i] = W25Q_Page_Write[i];
	}

	/////////////////////////////////////////////////////////////////////

	for(int i=0; i<Page_Size; i++){
		if(i< Repeater_Number){
			W25Q_Page_Write[i] = REPEATER_Acc_Set_Data[i];
		}
		else{
			W25Q_Page_Write[i] = 0;
		}
	}

	W25qxx_WritePage(W25Q_Page_Write, Repeater_Acc_Set_Address ,0,Page_Size);
	for(int i=0; i<Page_Size; i++){
		W25Q_Page_Read[i] = 0;
	}
	W25qxx_ReadPage(W25Q_Page_Read, Repeater_Acc_Set_Address, 0, Page_Size);

	for(int i=0; i<Repeater_Number; i++){
		REPEATER_Acc_Set_Data[i] = W25Q_Page_Read[i];
	}

}

//###################################################################################################################

//W25Q16 : 8192 Pages , 0 ~ 1891
uint32_t	W25qxx_PageToAddress(uint16_t Page_Num){

	uint32_t Return_Address;

	Return_Address = 0x000100 * Page_Num;

	return Return_Address;

}
