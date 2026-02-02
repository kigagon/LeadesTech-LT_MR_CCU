
#include "main.h"
#include "stm32u0xx.h"
#include "Internal_Flash.h"

uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/* Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0, PageError = 0;
uint32_t error_val ;

void Flash_Erase(uint32_t Address){

	// 1. 인터럽트 잠시 중단 (안전 제일)
	__disable_irq();

	// 2. 플래시 잠금 해제
	if(HAL_FLASH_Unlock() == HAL_OK)
	{
	    // 3. [핵심] 꼬여있는 Control Register 강제 초기화
	    // PG, PER, MER1 등 동작 비트를 모두 0으로 내림
	    FLASH->CR &= 0xFFFFF000; // 하위 비트들 강제 Clear (STM32U0 기준)

	    // 4. 상태 레지스터의 에러 플래그 클리어
	    // (참고: STM32는 에러 비트에 '1'을 써야 지워집니다. HAL 매크로가 이를 수행함)
	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PROGERR |
	                           FLASH_FLAG_WRPERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);

	    // 5. 확인 사살 (여전히 에러가 있는지 체크)
	    if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR))
	    {
	        // 여기까지 왔는데도 안 지워지면 하드웨어 리셋 필요
	         //printf("Error still exists!\n");
	    }
	}

	// 6. 인터럽트 복구
	__enable_irq();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = 0;
	EraseInitStruct.Page = GetPage(Address);
	EraseInitStruct.NbPages = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{

		error_val = HAL_FLASH_GetError();

		for(int i=0; i<10; i++){
			HAL_GPIO_WritePin(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(ERR_UART1_LED_GPIO_Port, ERR_UART1_LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
		}
	}

}


uint8_t Write_To_Flash(uint32_t Address, uint8_t *value, uint16_t Data_Length){

	uint64_t Save_Data_tmp = 0xf0f0f0f0f0f0f0f0;
	uint16_t Write_Data_Length = Data_Length / 8;
	uint8_t return_Val = 1;

	Flash_Erase(Address);

	if((Data_Length%8) == 0){
		Write_Data_Length = Data_Length/8;
	}
	else{
		Write_Data_Length = Data_Length/8 + 1;
	}

	for(int i=0; i<Write_Data_Length; i++ ){
		Save_Data_tmp = 0;
		for(int j=0; j<8;j++){
			Save_Data_tmp = (Save_Data_tmp << 8) | value[i*8+j];
		}

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Save_Data_tmp) == HAL_OK)
		{
		  Address = Address + 8;
		}
		else
		{
			return_Val = 0;
		}
	}

	HAL_FLASH_Lock();

	return return_Val;
}

void Flash_To_Read(uint32_t Address, uint8_t *value, uint16_t Data_Length){

	uint16_t Read_Data_len = (Data_Length / 8) + 1;
	uint64_t Read_Data_tmp;

	for(int i=0; i< Read_Data_len ; i++){
		Read_Data_tmp = *(__IO uint64_t *)Address;

		for(int j=0; j<8 ; j++){
			value[i*8 + j] = (Read_Data_tmp >> (64 - 8  - (j*8)) ) & 0xff;
		}

		Address= Address + 8;
	}
}

void Read_Info_Data(void){

	uint8_t Read_tmp_data[456];

	Flash_To_Read(Info_In_ADDR_FLASH, Read_tmp_data, 456);

	for(int i=0; i< 6; i++){
		Info_Data_Version[i] = Read_tmp_data[6+i];
	}

	for(int i=0; i<220; i++){
		REPEATER_Regster[i] = Read_tmp_data[(i*2)+13];
	}
}

void Read_Charge_Data(void){

	Flash_To_Read(Charge_In_ADDR_FLASH, REPEATER_Acc_Set_Data, Repeater_Number);
}
