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
MCU_State			state;  // structure that contains the current state of important variables
MCU_Instruction		instruction;
AS5600_TypeDef*		sensor;

volatile uint16_t	AS5600_analog;		// variable for angles received by ADC (automatic: dma)
uint16_t			AS5600_prev;		// variable that stores the last measurement
double				AS5600_pos_f64;
uint16_t			AS5600_pos;
int16_t				AS5600_delta_pos;

double				step_gain;	// - (backward) || + (forward) || 0 (idle) [uniform distribution between -1 and 1]

// define the following two in a struct received from spi
uint16_t			target = 2000;
int16_t				target_delta;
// <\>
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
void euler_method() {  // typical execution time ~45 us
	register double alpha = 1 / ((EULER_TAU / TIM5->CNT) + 1);
	AS5600_pos_f64 = (AS5600_analog * alpha) + ((1 - alpha) * AS5600_pos_f64);
	AS5600_delta_pos = (uint16_t)AS5600_pos_f64 - AS5600_pos;
	AS5600_pos = (uint16_t)AS5600_pos_f64;
	TIM5->CNT = 0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim != &htim10) { return; }
	//if (pre_euler_func) { (*pre_euler_func)(); }
	euler_method();  // update AS5600_pos, AS5600_delta_pos using the selected mode
	target_delta = target - AS5600_pos;
	target_delta = ABS(target_delta) < ABS(target_delta - 4096) ? target_delta : target_delta - 4096;

	// TODO: add continuous rotation space (-inf, inf) (software)
	// TODO: add defining rotation dir with SPI (this is obviously implicated with the previous TODO)
	// TODO: re-do: ease-in ease-out function
	// TODO: delete / re-do: pathfinding code
	// TODO: tune interrupt timing

	if (target_delta != 0) {
		HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, target_delta < 0);
		step_gain = MIN(ABS((double)target_delta / 1024), MIN(ABS(((double)target_delta + 1024) / 1024), ABS(((double)target_delta - 1024) / 1024)));  // all deltas greater than 1/8 rotation are met by a gain of 100%
		// optimize this or change the function
	} else { step_gain = 0; }
	return;
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
	sensor->positive_rotation_direction = AS5600_DIR_CW;
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
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);  // start timer_2
	HAL_TIM_Base_Start(&htim5);  // start timer_5
	// initialize AS5600 sensor
	while (AS5600_init(sensor) != HAL_OK) {}  // the sensor has to be on for the code to work

	// initialize the AS5600 position variable
	AS5600_get_angle(sensor, &AS5600_pos);
	AS5600_pos_f64 =	AS5600_pos;  // set the current angle to the most accurate value for the euler method
	AS5600_analog =		AS5600_pos;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// all DMA connectors are configured in cyclic mode so these functions have to be called once
	/*  DISABLED FOR TESTING
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&instruction, 16);  // start data receiving loop
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&state, 16);  // start data receiving loop
	*/

	// start receiving ADC data
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&AS5600_analog, 1);

	instruction.pulse_delay = 74; // 74;  // safe operating range is from 75us and up
	instruction.settings.micro_step = 3;
	instruction.settings.spread_mode = 0;

	// TODO: Add function to the INSTRUCT_GO interrupt pin that will start the stepping function

	set_motor_setting(&instruction);  // TODO: place correctly

	TIM5->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim10);  // start timer_10  (sensor interupt)
	HAL_GPIO_WritePin(STEPPER_NEN_GPIO_Port, STEPPER_NEN_Pin, 0);  // enable stepper (this is never undone)
	while (1) {
		if (step_gain < MIN_STEPPER_GAIN) { continue; }  // make sure that the step delay is never greater than 0.75 s
		// dir is set in interrupt
		register uint16_t pulse_delay = MIN_STEPPER_DELAY / step_gain;
		HAL_GPIO_WritePin(STEPPER_STP_GPIO_Port, STEPPER_STP_Pin, 1);
		delay_us(pulse_delay);
		HAL_GPIO_WritePin(STEPPER_STP_GPIO_Port, STEPPER_STP_Pin, 0);
		delay_us(pulse_delay);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
