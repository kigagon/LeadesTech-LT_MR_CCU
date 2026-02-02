/*
 * Analog_Process.c
 *
 *  Created on: Sep 25, 2025
 *      Author: root
 */


#include "main.h"
#include "Analog_Process.h"

//아날로그 정온식 저장 화일
uint8_t Analog_Temp_Data[220][Analog_Temp_Data_Num];

//아날로그 광전식 저장 화일
uint8_t Analog_Smoke_Data[220][Analog_Smoke_Data_Num];
uint8_t Process_value;

uint8_t Anal_Process(uint8_t Num ,uint8_t type , uint8_t data){


	Num = Num -1;
	if(type == 0x44){
		for(int i = 0 ; i < (Analog_Temp_Data_Num - 1)  ;i++){
			Analog_Temp_Data[Num][i] = Analog_Temp_Data[Num][i+1];
		}
		Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] = data;
/*
		if(((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) > 5)
				&((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) <= 4)){
			Process_value = (uint8_t)(data*1.06);
		}
		else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) > 3){
			Process_value =  (uint8_t)(data*1.03);
		}
		else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0])  > 1){
			Process_value =  (uint8_t)(data*1.00);
		}
		else{
			Process_value = (uint8_t)(data*0.97);
		}
*/
		if(Analog_Temp_Data[Num][0] == 0){
			Process_value = data;
		}
		else{
			if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) > 5){
				if(Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] > 65){
					Process_value = (uint8_t)(data*1.07);
				}
				else{
					Process_value = (uint8_t)(data*1.08);
				}
			}
			else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) == 5){
				if(Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] > 65){
					Process_value = (uint8_t)(data*1.07);
				}
				else{
					Process_value = (uint8_t)(data*1.08);
				}
			}
			else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) == 4){
				Process_value = (uint8_t)(data*1.06);
			}
			else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) == 3){
				Process_value = (uint8_t)(data*1.04);
			}
			else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) == 2){
				Process_value = (uint8_t)(data*1.0);
			}
			else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) == 1){
				Process_value = (uint8_t)(data*0.96);
			}
			else if((Analog_Temp_Data[Num][Analog_Temp_Data_Num-1] - Analog_Temp_Data[Num][0]) < 1){
				Process_value = (uint8_t)(data*0.92);
			}
		}

	}
	else if((type == 0x40)|(type == 0x41)|(type == 0x42)|(type == 0x43)){
		Process_value = data;
	}


	return Process_value;
}
