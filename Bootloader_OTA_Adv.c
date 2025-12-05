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
#include "FLASH.h"
#include <stdio.h>  
#include <string.h>  
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR_APP_PROGRAM 0x0800C800  
#define FLASH_END 0x08020000u                         // 128KB Flash
#define PAGE_SIZE 0x400  															// 1KB flash page
#define MAX_FW_SIZE (FLASH_END - ADDR_APP_PROGRAM)
#define UART_TIMEOUT_MS 5000  	
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint32_t rd_le32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

typedef void (*pFunction)(void);

static void enter_to_app(void)
{
    uint32_t app_msp = *(__IO uint32_t*)(ADDR_APP_PROGRAM + 0);  //0x20005000
    uint32_t app_rst = *(__IO uint32_t*)(ADDR_APP_PROGRAM + 4);
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
		HAL_UART_DeInit(&huart1);
    HAL_RCC_DeInit();
    HAL_DeInit();
    SCB->VTOR = ADDR_APP_PROGRAM; __DSB(); __ISB();
    __set_MSP(app_msp);
    __enable_irq();
    ((pFunction)app_rst)();
}

//======================kiem tra app hop le================================== */
static int Is_App_Valid(void)
{
    uint32_t sp = *(__IO uint32_t*)ADDR_APP_PROGRAM;               // stack pointer
    uint32_t reset = *(__IO uint32_t*)(ADDR_APP_PROGRAM + 4);		 	// reset handler
    if (sp == 0xFFFFFFFFU || reset == 0xFFFFFFFFU)
        return 0;    // ko hop le
    return 1;				 // hop le
}

//====================== Checksum (tinh tong don gian) ====================== */
static uint32_t Bootloader_CalcChecksum(uint8_t *data, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; i++) sum += data[i];
    return sum;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   //PA0 = 1

  /* ==== Cho và gui BL_READY, nhan header ==== */
	uint8_t header[8];
  uint32_t t0 = HAL_GetTick();
  uint32_t timeout_ms = UART_TIMEOUT_MS;
  int flag_header = 0;
  uint8_t ready_msg[] = "BL_READY\r\n";
  
  while ((HAL_GetTick() - t0) < timeout_ms) {
		
      HAL_UART_Transmit(&huart1, ready_msg, sizeof(ready_msg) - 1, 100);

      if (HAL_UART_Receive(&huart1, header, 8, 500) == HAL_OK) {
          flag_header = 1;
          break;
      }
  }
	
  if (!flag_header) {
      if (Is_App_Valid()) enter_to_app();
  }

  /* ========================== giai ma header ============================ */
  uint32_t fw_size = rd_le32(&header[0]);
  uint32_t fw_sum = rd_le32(&header[4]);
  if (fw_size == 0 || fw_size > MAX_FW_SIZE) {
      uint8_t err_msg[] = "FW_SIZE_ERR\r\n";
      HAL_UART_Transmit(&huart1, err_msg, sizeof(err_msg) - 1, 1000);
      if (Is_App_Valid()) enter_to_app();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			while (1) { }
  }

  /* ============================= Xóa Flash ============================== */
  Flash_unlock();
  uint32_t erase_addr = ADDR_APP_PROGRAM;
  uint32_t pages = (fw_size + PAGE_SIZE - 1U) / PAGE_SIZE;
  for (uint32_t p = 0; p < pages; p++) {
      Flash_Erase(erase_addr);
      erase_addr += PAGE_SIZE;
  }
  Flash_lock();
  uint8_t ready_ack[] = "READY\r\n";
  HAL_UART_Transmit(&huart1, ready_ack, sizeof(ready_ack) - 1, 1000);

	/* ========================= Nhân và ghi firmware ======================= */
  uint8_t rx_buf[256];
  uint32_t addr = ADDR_APP_PROGRAM;
  uint32_t remaining = fw_size;
  uint32_t calc_sum = 0;
  uint8_t write_fail = 0;
  Flash_unlock();
  while (remaining > 0) {
      uint16_t chunk = (remaining > sizeof(rx_buf)) ? sizeof(rx_buf) : (uint16_t)remaining;
      if (HAL_UART_Receive(&huart1, rx_buf, chunk, UART_TIMEOUT_MS) != HAL_OK) {
          uint8_t err_msg[] = "RX_ERR\r\n";
          HAL_UART_Transmit(&huart1, err_msg, sizeof(err_msg) - 1, 1000);
          write_fail = 1;
          break;
      }
			// ghi vao Flash
      Flash_write(addr, rx_buf, chunk);
			// tinh checksum
      calc_sum += Bootloader_CalcChecksum(rx_buf, chunk);
			// update status
      addr += chunk;
      remaining -= chunk;
  }
  Flash_lock();

  /* ====================== xu li lôi sau khi nhan firmware ========================= */
  if (write_fail || remaining > 0) {
      uint8_t abort_msg[] = "ABORT\r\n";
      HAL_UART_Transmit(&huart1, abort_msg, sizeof(abort_msg) - 1, 200);
      NVIC_SystemReset();
      while (1) { }
  }

	/* ===================== Kiem tra Checksum và enter_to_app ======================== */
	if (calc_sum == fw_sum) {
      const char *ok = "FW_OK\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)ok, strlen(ok), 100);
      HAL_Delay(50); 

      enter_to_app();
    } 
    else {
        const char *fail = "CRC_FAIL\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)fail, strlen(fail), 100);
        if (Is_App_Valid()) {
             enter_to_app();
        }

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        while(1) {}
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
