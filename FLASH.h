#ifndef __FLASH_H
#define __FLASH_H
#include <stm32f1xx_hal.h>

void Flash_lock();

void Flash_unlock();

void Flash_Erase(uint32_t addr);

void Flash_write(uint32_t addr, uint8_t *mData, uint16_t len);

void Flash_read(uint32_t addr, uint8_t *mData, uint16_t len);

#endif