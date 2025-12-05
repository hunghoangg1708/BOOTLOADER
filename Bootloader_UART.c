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
#include <string.h>
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR_APP_PROGRAM 		 0x0800C800   // d/c bat dau cua Application
#define PAGE_SIZE            0x400        // 1KB flash page
#define MAX_FW_SIZE          (64*1024)    // toi da 64KB firmware
#define UART_TIMEOUT_MS      5000         // 5 giây timeout
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
typedef void (*pFunction)(void);

//====================== Jump sang APP ====================== */
void enter_to_app(){
	
	HAL_RCC_DeInit();
	HAL_DeInit();

  SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk | 
                  SCB_SHCSR_BUSFAULTENA_Msk  | 
                   SCB_SHCSR_MEMFAULTENA_Msk );

	__set_MSP(*(__IO uint32_t*) ADDR_APP_PROGRAM);

	pFunction app_entry = (pFunction)(*(__IO uint32_t*)(ADDR_APP_PROGRAM + 4));
	app_entry();
}

//======================kiem tra app hop le====================== */
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
    for (uint32_t i = 0; i < len; i++)
        sum += data[i];
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
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);   // bat led booloader
	
		// Gui t/b san sàng qua UART
    const char *msg_ready = "BL_READY\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg_ready, strlen(msg_ready), HAL_MAX_DELAY);

    // Buffer header (8 byte: fw_size + fw_checksum)
    uint8_t header[8];

    // Cho nhan header trong 5 giây
    if (HAL_UART_Receive(&huart1, header, 8, UART_TIMEOUT_MS) != HAL_OK) {
        // Không nhan dc --> chay app cu
        if (Is_App_Valid()) enter_to_app();
    }
    else {   //thanh cong
        
        uint32_t fw_size = *((uint32_t*)&header[0]); // 4 byte firmware
        uint32_t fw_sum  = *((uint32_t*)&header[4]); // 4 byte checksum

        if (fw_size == 0 || fw_size > MAX_FW_SIZE) {
            const char *err = "FW_SIZE_ERR\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
            if (Is_App_Valid()) enter_to_app();
        }

        // ===================== Xóa Flash cu =====================
        Flash_unlock();
        uint32_t erase_addr = ADDR_APP_PROGRAM;
        uint32_t pages = (fw_size + PAGE_SIZE - 1) / PAGE_SIZE;   //tính so trang can xoa
        for (uint32_t p = 0; p < pages; p++) {
            Flash_Erase(erase_addr);
            erase_addr += PAGE_SIZE;
        }
        Flash_lock();

        // ===================== Nhan và ghi Firmware =====================
        uint8_t rx_buf[256];
        uint32_t addr = ADDR_APP_PROGRAM;
        uint32_t remaining = fw_size;
        uint32_t calc_sum = 0;

        while (remaining > 0) {
            uint16_t chunk = (remaining > sizeof(rx_buf)) ? sizeof(rx_buf) : remaining;

            if (HAL_UART_Receive(&huart1, rx_buf, chunk, UART_TIMEOUT_MS) != HAL_OK) {
                const char *err = "RX_ERR\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
                if (Is_App_Valid()) enter_to_app();
            }

            Flash_unlock();
            Flash_write(addr, rx_buf, chunk);
            Flash_lock();

            calc_sum += Bootloader_CalcChecksum(rx_buf, chunk);
            addr += chunk;
            remaining -= chunk;
        }

        // ===================== Kiem tra Checksum =====================
        if (calc_sum == fw_sum) {
            const char *ok = "FW_OK\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)ok, strlen(ok), HAL_MAX_DELAY);
            HAL_Delay(100);

            // nhay sang application moi
            enter_to_app();
        } 
        else {
            const char *fail = "CRC_FAIL\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)fail, strlen(fail), HAL_MAX_DELAY);
            if (Is_App_Valid()) enter_to_app();
        }
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(500);
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
