/*
 * Analog_Process.h
 *
 *  Created on: Sep 25, 2025
 *      Author: root
 */

#ifndef INC_ANALOG_PROCESS_H_
#define INC_ANALOG_PROCESS_H_

#define Analog_Temp_Data_Num	15
#define Analog_Smoke_Data_Num	3

//아날로그 정온식 저장 화일
extern uint8_t Analog_Temp_Data[220][Analog_Temp_Data_Num];

//아날로그 광전식 저장 화일
extern uint8_t Analog_Smoke_Data[220][Analog_Smoke_Data_Num];

uint8_t Anal_Process(uint8_t Num ,uint8_t type , uint8_t data);

#endif /* INC_ANALOG_PROCESS_H_ */
