/* USER CODE BEGIN Header */
// STM32F411ceu6
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
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
AS5600_TypeDef* sensor;

volatile uint16_t AS5600_analog;		// variable for angles received by ADC (automatic: dma)
volatile uint16_t AS5600_i2c;					// variable for angles received by I2C (manual)
uint16_t* euler_next = &AS5600_analog;	// points to a the variable that will be added using the euler method (either: analog or i2c)
double AS5600_pos_f64;
uint16_t AS5600_pos;
int16_t AS5600_delta_pos;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dma_i2c_callback(DMA_HandleTypeDef* hdma) {
	HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&AS5600_i2c, 2);
}

void delay_us(uint32_t n) { TIM2->CNT = 0; while(TIM2->CNT < n); }
void until_us(uint32_t n) { while(TIM2->CNT < n); }  // this will wait until the count register is set to a specific value this allows code to be ran while waiting
void set_motor_setting(MCU_Instruction* instruction) {
	GPIOA->ODR &= RST;
	switch(instruction->settings.micro_step) {
	case 0: GPIOA->ODR |= M2; break;
	case 1: GPIOA->ODR |= M4; break;
	case 2: GPIOA->ODR |= M8; break;  // default is 1/8 micro stepping
	case 3: GPIOA->ODR |= M16; break;
	}
	HAL_GPIO_WritePin(STEPPER_SRD_GPIO_Port, STEPPER_SRD_Pin, instruction->settings.spread_mode);
}
void euler_method(uint16_t next) {  // typical execution time ~45 us
	AS5600_pos_f64 += (next - AS5600_pos_f64) / (AS5600_EULER_TAU / (double)TIM5->CNT);  // TIM5->CNT holds the time since the last addition in us
	AS5600_delta_pos = (uint16_t)AS5600_pos_f64 - AS5600_pos;
	AS5600_pos = (uint16_t)AS5600_pos_f64;
	TIM5->CNT = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	sensor = AS5600_new();
	sensor->i2c_handle = &hi2c1;
	sensor->dir_port = AS5600_DIR_GPIO_Port;
	sensor->dir_pin = AS5600_DIR_Pin;
	sensor->positive_rotation_direction = AS5600_DIR_CCW;
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);  // start timer_2
	HAL_TIM_Base_Start(&htim5);  // start timer_5

	// initialize AS5600 sensor
	while (AS5600_init(sensor) != HAL_OK) {}  // the sensor has to be on for the code to work

	// initialize the AS5600 position variable
	AS5600_get_angle(sensor, &AS5600_pos);
	AS5600_pos_f64 = AS5600_pos;  // set the current angle to the most accurate value for the euler method
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// all DMA connectors are configured in cyclic mode so these functions have to be called once
	/*  DISABLED FOR STEPPER DRIVER TESTING
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&instruction, 16);  // start data receiving loop
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&state, 16);  // start data receiving loop
	*/

	// start receiving ADC data
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&AS5600_analog, 1);
	// HAL_I2C_Mem_Read_DMA exec time 0 - 5 us??
	HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&AS5600_i2c, 2);  // only makes one request??
	hdma_i2c1_rx.XferCpltCallback = dma_i2c_callback;

	while (1) {  // run following tests: execution time and cyclic requests?
	}


	uint16_t target = 0;  // define this in a struct received from spi

	uint16_t target_delta;
	while (1) {
		euler_method(*euler_next);  // update AS5600_pos, AS5600_delta_pos using the selected mode
		target_delta = target - AS5600_pos;
		if (euler_next == AS5600_analog && target_delta < AS5600_ADC_ERROR_MARGIN) {  // switch to I2C mode
			HAL_ADC_Stop_DMA(&hadc1);
			euler_next = &AS5600_i2c;
			HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&AS5600_i2c, 2);  // only makes one request??
			continue;
		} else if (euler_next == AS5600_i2c && target_delta > AS5600_ADC_ERROR_MARGIN) {  // switch to ADC mode

			euler_next = &AS5600_analog;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&AS5600_analog, 1);
			continue;
		}
		// TODO: add condition to switch back to analog mode <<<<<<<<<<<<<<<<<<<<<<<

	}

	// buffers
	uint64_t	iter			= 0;
	int8_t		mult			= 0;
	uint64_t	pulse_delay_us	= 0;

	instruction.steps = -100000000;
	instruction.pulse_delay = 474; // 74;  // safe operating range is from 75us and up
	instruction.settings.micro_step = 3;
	instruction.settings.spread_mode = 0;

	while (1) {
		if (instruction.steps == 0) { continue; }
		set_motor_setting(&instruction);

		state.job = instruction.steps;
		pulse_delay_us = instruction.pulse_delay + 1;
		instruction.steps = 0;

		HAL_GPIO_WritePin(STEPPER_NEN_GPIO_Port, STEPPER_NEN_Pin, 0);
		HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, state.job > 0);
		// capture the current job (this can change via interrupt)
		mult = state.job > 0 ? 1 : -1;
		iter = abs_64(state.job);
		for (uint64_t i = 0; i < iter; i++) {  // iterate this job for max 4096 iterations at the time before SPI receive
			// update variables first because the SPI DMA transmit has a almost 100% chance to happen during pulse delay
			state.pos += mult;
			state.job -= mult;
			HAL_GPIO_WritePin(STEPPER_STP_GPIO_Port, STEPPER_STP_Pin, 1);
			delay_us(pulse_delay_us);
			HAL_GPIO_WritePin(STEPPER_STP_GPIO_Port, STEPPER_STP_Pin, 0);
			delay_us(pulse_delay_us);
		}
		HAL_GPIO_WritePin(STEPPER_NEN_GPIO_Port, STEPPER_NEN_Pin, 1);
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
