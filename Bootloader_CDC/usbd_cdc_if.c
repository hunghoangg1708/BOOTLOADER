/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS;

/* FIFO nhận (kích thước là lũy thừa của 2) */
#define RX_FIFO_SZ 4096u

static volatile uint8_t  rx_fifo[RX_FIFO_SZ];
static volatile uint16_t rx_w = 0, rx_r = 0;

/* PROTOTYPE nội bộ */
static void     fifo_push(const uint8_t* src, uint16_t n);
static uint16_t fifo_pop (uint8_t* dst, uint16_t n);

/* USER CODE END PV */

/* ==== Các hàm tiện ích dùng cho bootloader ==== */

bool CDC_IsConfigured(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

void CDC_WaitTxDone(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  while (hcdc && hcdc->TxState) {
    if ((HAL_GetTick() - t0) >= timeout_ms) {
      break;
    }
  }
}

static inline uint16_t fifo_level(void)
{
  return (uint16_t)(rx_w - rx_r);
}

static void fifo_push(const uint8_t* src, uint16_t n)
{
  while (n--) {
    /* Nếu đầy gần hết thì bỏ phần còn lại (không đếm tràn nữa) */
    if (fifo_level() >= RX_FIFO_SZ - 1u) {
      return;
    }
    rx_fifo[rx_w & (RX_FIFO_SZ - 1u)] = *src++;
    rx_w++;
  }
}

void CDC_FlushRx(void)
{
  rx_r = rx_w;
}

static uint16_t fifo_pop(uint8_t* dst, uint16_t n)
{
  uint16_t got = 0;
  while (got < n && rx_r != rx_w) {
    dst[got++] = rx_fifo[rx_r & (RX_FIFO_SZ - 1u)];
    rx_r++;
  }
  return got;
}

HAL_StatusTypeDef CDC_ReadExact(uint8_t* dst, uint16_t need, uint32_t timeout_ms)
{
  uint32_t t0  = HAL_GetTick();
  uint16_t got = 0;

  /* Đợi USB được cấu hình */
  while (!CDC_IsConfigured()) {
    if ((HAL_GetTick() - t0) >= timeout_ms)
      return HAL_TIMEOUT;
  }

  /* Đọc đủ need byte từ FIFO do CDC_Receive_FS đẩy vào */
  while (got < need) {
    got += fifo_pop(dst + got, (uint16_t)(need - got));
    if (got >= need)
      break;

    if ((HAL_GetTick() - t0) >= timeout_ms)
      return HAL_TIMEOUT;
  }
  return HAL_OK;
}

USBD_StatusTypeDef CDC_Write(const uint8_t* src,
                             uint16_t       len,
                             uint32_t       timeout_ms)
{
  uint32_t t0 = HAL_GetTick();

  while (1) {
    if (!CDC_IsConfigured())
      return USBD_FAIL;

    USBD_StatusTypeDef st = CDC_Transmit_FS((uint8_t*)src, len);
    if (st == USBD_OK) {
      CDC_WaitTxDone(timeout_ms);
      return USBD_OK;
    }

    if ((HAL_GetTick() - t0) >= timeout_ms)
      return USBD_BUSY;
  }
}

/* ===================================================================== */
/*                   PHẦN CALLBACK CỦA THƯ VIỆN CDC                      */
/* ===================================================================== */

/* Create buffer for reception and transmission */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private function prototypes for USB CDC */
static int8_t CDC_Init_FS   (void);
static int8_t CDC_DeInit_FS (void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* CDC interface callback structure */
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  */
static int8_t CDC_Init_FS(void)
{
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);   // arm lần nhận đầu tiên
  return (USBD_OK);
}

/**
  * @brief  DeInitializes the CDC media low layer
  */
static int8_t CDC_DeInit_FS(void)
{
  /* Không cần làm gì thêm cho bootloader */
  return (USBD_OK);
}

/**
  * @brief  Manage the CDC class requests
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  (void)pbuf;
  (void)length;

  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SET_LINE_CODING:
    case CDC_GET_LINE_CODING:
    case CDC_SEND_BREAK:
    case CDC_SET_CONTROL_LINE_STATE:
    default:
      break;
  }

  return (USBD_OK);
}

/**
  * @brief  Data received over USB OUT endpoint
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  fifo_push(Buf, (uint16_t)(*Len));         // Đưa dữ liệu vào FIFO
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Buf); // Arm lại OUT endpoint
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

/**
  * @brief  Transmit data over USB IN endpoint
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  if (hcdc == NULL || hcdc->TxState != 0) {
    return USBD_BUSY;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  return result;
}
