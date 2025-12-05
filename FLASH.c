#include "FLASH.h"


void Flash_lock(){
	HAL_FLASH_Lock();
}

void Flash_unlock(){
	HAL_FLASH_Unlock();
}

void Flash_Erase(uint32_t addr){
	FLASH_EraseInitTypeDef pErase;
	pErase.NbPages = 1;
	pErase.PageAddress = addr;
	pErase.TypeErase = FLASH_TYPEERASE_PAGES;
	uint32_t PError;
	HAL_FLASHEx_Erase(&pErase, &PError);
}

void Flash_write(uint32_t addr, uint8_t *mData, uint16_t len){
	uint8_t status = 0x00;
	if(len % 2 == 1){
		status = 1;
		len--;  // dua ve so chan 
	}
	for(uint32_t i = 0; i < len; i+=2){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, *(mData+i) | (uint16_t)(*(mData+i+1) << 8));
	}
	if(status == 1){   //so le
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + len, *(mData+len) | (uint16_t)((0xFF) << 8));
	}
}

void Flash_read(uint32_t addr, uint8_t *mData, uint16_t len){
	for(uint32_t i = 0; i < len; i+=2){
		volatile uint32_t  *address = (volatile uint32_t *)(addr + i);
		uint16_t temp_data = *address;
		mData[i] = (uint8_t)temp_data;
		mData[i+1] = (uint8_t)temp_data >> 8;
	}
}