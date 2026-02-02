/*
 * ReadWriteFlash.c
 *
 *  Created on: Sep 19, 2025
 *      Author: root
 */

/*
 *
 * Sector Size : 4096byte
 * Sector 0 : 시스템 정보 예약
 *
 * Sector 1 : 중계기 1 등록정보
 *
 *
 */

#include "main.h"
#include "w25qxx.h"
#include "w25qxxConf.h"
#include "ReadWriteFlash.h"



int Write_Regi_data(uint8_t Num, uint8_t *arr, int size){

	int Succsee = 1, Failure = 0 , Data_Check = 0;
	uint8_t Temp_Sector[4096];

	for(int i = 0; i< 4096; i++){
		Temp_Sector[i] = 0xff;
	}
	if((Num >= 2)|(Num == 0) ){

	}
	else{
		for(int i = 0; i< size; i++){
			Temp_Sector[i] = arr[i];
		}
		W25qxx_EraseSector(Num);
		W25qxx_WriteSector(arr, Num, 0, 4096);
		for(int i = 0; i< 4096; i++){
			Temp_Sector[i] = 0x00;
		}
		W25qxx_ReadSector(Temp_Sector, Num, 0, 4096);
	}

	for(int i=0 ; i<size ; i++){
		if(Temp_Sector[i] == arr[i]){
			Data_Check = 1;
		}
		else{
			Data_Check = 0;
		}

		if(Data_Check == 0){
			break;
		}
	}

	if(Data_Check == 1){
		return Succsee;
	}
	else{
		return Failure;
	}
}

