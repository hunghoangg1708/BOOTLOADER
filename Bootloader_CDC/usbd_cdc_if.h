/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v2.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include <stdbool.h>

/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* API dùng cho bootloader */
uint8_t           CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
HAL_StatusTypeDef CDC_ReadExact(uint8_t* dst, uint16_t need, uint32_t timeout_ms);
USBD_StatusTypeDef CDC_Write(const uint8_t* src, uint16_t len, uint32_t timeout_ms);
bool              CDC_IsConfigured(void);
void              CDC_WaitTxDone(uint32_t timeout_ms);
void              CDC_FlushRx(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
