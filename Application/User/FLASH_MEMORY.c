/*
 * FLASH_MEMORY.c
 *
 *  Created on: Jul 3, 2023
 *      Author: user
 */

#include "FLASH_MEMORY.h"
#include "stm32f4xx_hal.h"
#include "CAN.h"

uint32_t Address_CHECK              = 0x08060000;
uint32_t Address_ZERO               = 0x08060004;
uint32_t Address_ID                 = 0x08060008;
uint32_t Adrress_EPT_Neutral        = 0x0806000C;
uint32_t Adrress_EPT_STAGTE2        = 0x08060010;

uint32_t Address_ZERO_buffer        = 0x08040000;
uint32_t Address_ID_buffer          = 0x08040004;
uint32_t Adrress_EPT_Neutral_buffer = 0x08040008;
uint32_t Adrress_EPT_STAGTE2_buffer = 0x0804000C;

uint16_t NEW_DATA_CHECK=0;
uint32_t ZERO_SETTING_POINT;

int ZERO_SETTING=0;
int ID_SETTING=0;
int EPT_SETTING=0;

uint8_t ID;
uint8_t EPT_Neutral;
uint8_t EPT_STAGE2;



void FLASH_MEMORY()
{
   NEW_DATA_CHECK     = *(__IO uint16_t*)Address_CHECK;
   ZERO_SETTING_POINT = *(__IO uint16_t*)Address_ZERO;
   ID                 = *(__IO uint16_t*)Address_ID;
   EPT_Neutral        = *(__IO uint16_t*)Adrress_EPT_Neutral;
   EPT_STAGE2         = *(__IO uint16_t*)Adrress_EPT_STAGTE2;


   if(NEW_DATA_CHECK==0x00000000)
   {
	    ZERO_SETTING_POINT = *(__IO uint16_t*)Address_ZERO;
	    ID                 = *(__IO uint16_t*)Address_ID;
	    EPT_Neutral        = *(__IO uint16_t*)Adrress_EPT_Neutral;
	    EPT_STAGE2         = *(__IO uint16_t*)Adrress_EPT_STAGTE2;

	    FLASH_Erase_Sector(FLASH_SECTOR_7, FLASH_VOLTAGE_RANGE_3); //섹터지우기 (지우고 난 다음에 쓸 수 있음)
	    FLASH_Erase_Sector(FLASH_SECTOR_6, FLASH_VOLTAGE_RANGE_3);

   }
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ZERO_buffer, ZERO_SETTING_POINT); //memory to 0x08040004 , 쓰기
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ID_buffer, ID); //memory to 0x08040004
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Adrress_EPT_Neutral_buffer, EPT_Neutral);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Adrress_EPT_STAGTE2_buffer, EPT_STAGE2);
		ZERO_SETTING_POINT = *(__IO uint16_t*)Address_ZERO_buffer;
		ID                 = *(__IO uint16_t*)Address_ID_buffer;
		EPT_Neutral        = *(__IO uint16_t*)Adrress_EPT_Neutral_buffer;
	    EPT_STAGE2         = *(__IO uint16_t*)Adrress_EPT_STAGTE2_buffer;
}

void FLASH_MEMORY_SETTING()
{
	if(ZERO_SETTING==1)
	{
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ZERO, ENCODER_VALUE);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ID, *(__IO uint16_t*)Address_ID_buffer);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Adrress_EPT_Neutral, *(__IO uint16_t*)Adrress_EPT_Neutral_buffer);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Adrress_EPT_STAGTE2, *(__IO uint16_t*)Adrress_EPT_STAGTE2_buffer);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_CHECK, 0x00000000);
		ZERO_SETTING=0;
		HAL_FLASH_Lock();
		NVIC_SystemReset();
	}

	if(ID_SETTING==1)
	{
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ZERO, *(__IO uint16_t*)Address_ZERO_buffer);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ID, ID);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_CHECK, 0x00000000);
		ID_SETTING=0;
		HAL_FLASH_Lock();
		NVIC_SystemReset();
	}
	if(EPT_SETTING==1)
	{
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Adrress_EPT_Neutral, EPT_Neutral);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Adrress_EPT_STAGTE2, EPT_STAGE2);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ZERO, *(__IO uint16_t*)Address_ZERO_buffer);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_ID, *(__IO uint16_t*)Address_ID_buffer);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_CHECK, 0x00000000);
		EPT_SETTING=0;
		HAL_FLASH_Lock();
		NVIC_SystemReset();
	}
}



