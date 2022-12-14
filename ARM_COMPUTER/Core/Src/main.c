/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>

#include "uart_buffer.h"
#include "crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t RX_data[896];  // 32 instructions
uart_ibuf* RX_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	RX_data = new_uart_ibuf(&huart2, 1024);  // starts receiving

	// motor_count starts counting from 0
	uint8_t motor_count = 0;  // TODO: make code to find motor count

	{  // anonymous scope so that temporary variables are cleaned up
		uint32_t baud = 9600;  // default baud
		CTRL_Handshake init;

		HANDSHAKE:  // jmp to label in the case baud is changed (redo handshake)

		memset((void*)&init, 0, 6);  // set to 0
		uint16_t crc = 0xffff;

		while (init.crc != crc) {  // handshake
			if (uart_ibuf_align(RX_data, SYNC_BYTE)) { continue; }
			if (uart_ibuf_read(RX_data, &init, 6)) { continue; }
			crc = crc16_dnp(&init, 4);
		}  // TODO: add CRC error correction

		init.motor_count = motor_count;  // set motor count and send message back
		HAL_UART_Transmit(&huart2, (uint8_t*)&init, 6, 10);

		if (init.init_0) {}  // TODO: move all motors to their 0 position
		if (init.baud != baud) {
			// TODO: set new baud and go back to handshake
			baud = init.baud;
			goto HANDSHAKE;
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// HAL_UART_Transmit(&huart2, (uint8_t*)&instruction, 28, 100);  // 10ms timeout is too little!!!!
	uart_ibuf_reset(RX_data);
	MCU_Instruction instruction;
	MCU_State state;
	uint8_t return_code;

	instruction.target = 2000;
	instruction.max_vel = 1;
	instruction.max_acc = 1;
	instruction.micro_step = 3;
	instruction.srd_mode = 0;
	instruction.action = 0xf;  // all
	instruction.dir = 0;  // fastest
	instruction.id = 0;
	instruction.instrution_id = 0;
	instruction.crc = crc16_dnp(&instruction, 30);


	while (1) {
		if (uart_ibuf_align(RX_data, SYNC_BYTE)) { continue; }
		if (uart_ibuf_read(RX_data, &instruction, 32)) { continue; }

		return_code = 0;
		if (instruction.crc != crc16_dnp(&instruction, 30)) {
			// TODO: error correction
			// if not fix-able \/
			return_code |= RETURN_CRC_ERROR;
			HAL_UART_Transmit(&huart2, (uint8_t*)&return_code, 1, 10);
			continue;  // do not execute instruction
		}
		// TODO: check if instruction is valid, and try fixing it
		// return_code |= RETURN_ERROR;
		// HAL_UART_Transmit(&huart2, (uint8_t*)&return_code, 1, 10);
		// continue;
		// if not valid ^
		// if fixed \/
		// return_code |= RETURN_ERROR_FIXED;
		return_code |= RETURN_OK;
		HAL_UART_Transmit(&huart2, (uint8_t*)&return_code, 1, 10);

		// TODO: test this
		// TODO: make messages more flexible for other motor drivers
		do {
			HAL_GPIO_WritePin(CS_0_GPIO_Port, CS_0_Pin, 0);
			HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&instruction, (uint8_t*)&state, 32, 100);
			HAL_GPIO_WritePin(CS_0_GPIO_Port, CS_0_Pin, 1);
			HAL_Delay(10);  // give mcu time to react
		} while (HAL_GPIO_ReadPin(TEST_CF_GPIO_Port, TEST_CF_Pin));
		if (instruction.action & OVERRIDE) {
			HAL_GPIO_WritePin(C_INT_GPIO_Port, C_INT_Pin, 0);
			HAL_Delay(5);  // give mcu time to react
			HAL_GPIO_WritePin(C_INT_GPIO_Port, C_INT_Pin, 1);
		}
		if (instruction.action & POLL) {
			return_code = SYNC_BYTE;
			HAL_UART_Transmit(&huart2, (uint8_t*)&return_code, 1, 10);
			HAL_UART_Transmit(&huart2, (uint8_t*)&state, 32, 100);
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
