/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "FLASH.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR_APP_PROGRAM  0x0800C800u   // Địa chỉ bắt đầu Application
#define FLASH_END         0x08010000u   // Cuối vùng Flash
#define PAGE_SIZE         0x400u        // 1 KB / page
#define MAX_FW_SIZE       (FLASH_END - ADDR_APP_PROGRAM)
#define UART_TIMEOUT_MS   5000u         // 5 giây timeout cho nhận dữ liệu
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static int      Is_App_Valid(void);
static void     BL_JumpToApp(void);
static uint32_t Bootloader_CalcChecksum(uint8_t *data, uint32_t len);
static inline   uint32_t rd_le32(const uint8_t* p);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Đọc 4 byte little-endian thành uint32_t */
static inline uint32_t rd_le32(const uint8_t* p)
{
    return  (uint32_t)p[0]
          | ((uint32_t)p[1] << 8)
          | ((uint32_t)p[2] << 16)
          | ((uint32_t)p[3] << 24);
}

static int Is_App_Valid(void)
{
    uint32_t sp = *(uint32_t*)(ADDR_APP_PROGRAM + 0);
    uint32_t rh = *(uint32_t*)(ADDR_APP_PROGRAM + 4);

    if (sp < 0x20000000U || sp > 0x20005000U) return 0;
    if (rh < ADDR_APP_PROGRAM || rh > FLASH_END) return 0;

    return 1;
}

static uint32_t Bootloader_CalcChecksum(uint8_t *data, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; i++)
        sum += data[i];
    return sum;
}


static void BL_JumpToApp(void)
{
    uint32_t app_msp = *(__IO uint32_t*)(ADDR_APP_PROGRAM + 0);
    uint32_t app_rst = *(__IO uint32_t*)(ADDR_APP_PROGRAM + 4);

    if (!Is_App_Valid()) {
        while (1) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
            HAL_Delay(500);
        }
    }

    __disable_irq();

    CDC_WaitTxDone(100);
    USBD_DeInit(&hUsbDeviceFS);
    __HAL_RCC_USB_CLK_DISABLE();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    HAL_RCC_DeInit();
    HAL_DeInit();

    SCB->VTOR = ADDR_APP_PROGRAM; __DSB(); __ISB();

    __set_MSP(app_msp);

    __enable_irq();

    ((void (*)(void))app_rst)();

    while (1) { }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Initialize USB CDC */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);   // bật LED báo đang ở bootloader

  /* ==== Chờ BL_READY/HEADER 1 lần ==== */
  uint8_t  header[8];
  uint32_t t0         = HAL_GetTick();
  uint32_t timeout_ms = 5000;
  int      got_header = 0;

  while ((HAL_GetTick() - t0) < timeout_ms) {
      if (CDC_IsConfigured()) {
          CDC_Write((uint8_t*)"BL_READY\r\n", 10, 100);
          CDC_WaitTxDone(20);
      }

      if (CDC_ReadExact(header, 8, 50) == HAL_OK) {
          got_header = 1;
          CDC_FlushRx();
          break;
      }
      HAL_Delay(30);
  }

  if (!got_header) {
      if (Is_App_Valid()) {
          BL_JumpToApp();
      }
  }

  /* ==== Parse header (little-endian) ==== */
  uint32_t fw_size = rd_le32(&header[0]);
  uint32_t fw_sum  = rd_le32(&header[4]);

  if (fw_size == 0 || fw_size > MAX_FW_SIZE) {
      CDC_Write((const uint8_t*)"FW_SIZE_ERR\r\n", 13, 1000);
      if (Is_App_Valid()) {
          BL_JumpToApp();
      }
      while (1) {
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
          HAL_Delay(500);
      }
  }

  Flash_unlock();
  uint32_t erase_addr = ADDR_APP_PROGRAM;
  uint32_t pages      = (fw_size + PAGE_SIZE - 1U) / PAGE_SIZE;

  for (uint32_t p = 0; p < pages; p++) {
      Flash_Erase(erase_addr);
      erase_addr += PAGE_SIZE;
  }
  Flash_lock();

  CDC_Write((uint8_t*)"READY\r\n", 7, 1000);
  CDC_WaitTxDone(50);

  /* ==== Nhận và ghi firmware ==== */
  uint8_t  rx_buf[256];
  uint32_t addr      = ADDR_APP_PROGRAM;
  uint32_t remaining = fw_size;
  uint32_t calc_sum  = 0;
  uint8_t  write_fail = 0;

  Flash_unlock();
  while (remaining > 0) {
      uint16_t chunk = (remaining > sizeof(rx_buf)) ? sizeof(rx_buf)
                                                    : (uint16_t)remaining;

      if (CDC_ReadExact(rx_buf, chunk, UART_TIMEOUT_MS) != HAL_OK) {
          CDC_Write((const uint8_t*)"RX_ERR\r\n", 8, 1000);
          write_fail = 1;
          break;
      }

      if (Flash_write(addr, rx_buf, chunk) != HAL_OK) {
          CDC_Write((uint8_t*)"FLASH_ERR\r\n", 12, 200);
          write_fail = 1;
          break;
      }

      calc_sum += Bootloader_CalcChecksum(rx_buf, chunk);
      addr      += chunk;
      remaining -= chunk;
  }
  Flash_lock();

  if (write_fail || remaining > 0) {
      CDC_Write((uint8_t*)"ABORT\r\n", 7, 200);
      NVIC_SystemReset();
      while (1) { }
  }

  uint32_t sum_flash = 0;
  for (uint32_t a = ADDR_APP_PROGRAM; a < ADDR_APP_PROGRAM + fw_size; a++) {
      sum_flash += *(__IO uint8_t*)a;
  }

  /* ---- Kiểm tra vector + quyết định nhảy ---- */
  uint32_t sp      = *(__IO uint32_t*)(ADDR_APP_PROGRAM + 0);
  uint32_t rh      = *(__IO uint32_t*)(ADDR_APP_PROGRAM + 4);
  uint32_t rh_even = rh & ~1u;

  if (sum_flash == fw_sum &&
      sp >= 0x20000000u && sp <= 0x20005000u &&
      rh_even >= ADDR_APP_PROGRAM && rh_even < FLASH_END)
  {
      BL_JumpToApp();
  }
  else
  {
      while (1) {
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
          HAL_Delay(500);
      }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType    = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState          = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState          = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState      = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource     = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL        = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = GPIO_PIN_0;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
