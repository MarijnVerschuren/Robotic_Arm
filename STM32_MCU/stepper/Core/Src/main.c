/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
MCU_State state;  // structure that contains the current state of important variables
MCU_Instruction instruction;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t n) { TIM1->CNT = 0; while(TIM1->CNT < n); }
void MCU_Instruction_reset(MCU_Instruction* instruction) { instruction->steps = 0; instruction->pulse_delay = 0; }
uint8_t MCU_Instruction_bool(MCU_Instruction* instruction) { return instruction->steps != 0; }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	state.pos = 0;  // the current position
	state.job = 0;  // the future position
	MCU_Instruction_reset(&instruction);
  /* USER CODE END 1 */

  /* MCU Configuration-------------------------------------------------------- */

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);  // start timer_1
	// buffers
	uint64_t	iter			= 0;
	int8_t		mult			= 0;
	uint64_t	pulse_delay_us	= 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// all DM connectors are configured in cyclic mode so these functions have to be called once

	/*  DISABLED FOR STEPPER DRIVER TESTING
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&instruction, 16);  // start data receiving loop
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&state, 16);  // start data receiving loop
	*/

	// HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress, pData, Size)
	// TODO: ADD PWM / DIGITAL INPUT FROM AS5600 & ADD PROGRAMMING CAPABILITY <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	// some pin names have changed <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< (also add i2c support for mae(magnetic angle encoder) idealy with dma)
	while (1) {
		// TODO: FIX STEPPING CODE AND PIN NAMES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		if (!MCU_Instruction_bool(&instruction)) { continue; }

		state.job += instruction.steps;
		pulse_delay_us = instruction.pulse_delay + 1;
		MCU_Instruction_reset(&instruction);

		HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, 1);
		HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, state.job > 0);
		// capture the current job (this can change via interrupt)
		mult = state.job > 0 ? 1 : -1;
		iter = abs_64(state.job);
		for (uint64_t i = 0; i < iter; i++) {  // iterate this job for max 4096 iterations at the time before SPI receive
			HAL_GPIO_WritePin(STEPPER_CLK_GPIO_Port, STEPPER_CLK_Pin, 1);
			delay_us(pulse_delay_us);
			HAL_GPIO_WritePin(STEPPER_CLK_GPIO_Port, STEPPER_CLK_Pin, 0);
			delay_us(pulse_delay_us);
			state.pos += mult;
			state.job -= mult;
		}
		HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, 0);
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
