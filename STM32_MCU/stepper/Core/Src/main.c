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
MCU_State					state;					// structure that contains the current state of important variables
volatile MCU_Instruction	instruction;			// current instruction TODO: change -> to . and copy instruction into this variable <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
volatile Linked_List*		queue;
volatile MCU_Instruction	instruction_input;		// instruction directly received from SPI (this will be put into queue if there is space)
AS5600_TypeDef*				sensor;

volatile double				AS5600_pos_f64;			// accumulator
volatile double				step_gain;				// - (backward) || + (forward) || 0 (idle) [uniform distribution between -1 and 1]
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void status_parity(void) {
	state.n_status = ~state.status;
	state.status_parity =	(state.status & 0x1)		^ \
							((state.status >> 1) & 0x1)	^ \
							((state.status >> 2) & 0x1)	^ \
							((state.status >> 3) & 0x1);
}
void set_status(uint8_t stat)	{ state.status |= stat;	status_parity(); }
void reset_status(uint8_t stat)	{ state.status &= ~stat;status_parity(); }

void delay_us(uint32_t n) { TIM2->CNT = 0; while(TIM2->CNT < n); }
void until_us(uint32_t n) { while(TIM2->CNT < n); }  // this will wait until the count register is set to a specific value this allows code to be ran while waiting
void set_motor_setting(MCU_Instruction* instruction) {
	GPIOA->ODR &= RST;
	switch(instruction->micro_step) {
	case 0: GPIOA->ODR |= M2; break;
	case 1: GPIOA->ODR |= M4; break;
	case 2: GPIOA->ODR |= M8; break;  // default is 1/8 micro stepping
	case 3: GPIOA->ODR |= M16; break;
	}
	HAL_GPIO_WritePin(STEPPER_SRD_GPIO_Port, STEPPER_SRD_Pin, instruction->srd_mode);
}
void euler_method(void) {  // typical execution time ~45 us
	register uint16_t pos_diff = (AS5600_pos_f64 - state.raw_angle);  // rotation detection
	state.pos.rotation += pos_diff > 2048; state.pos.rotation -= pos_diff < -2048;
	register double alpha = 1 / ((EULER_TAU / TIM5->CNT) + 1);
	AS5600_pos_f64 = (state.raw_angle * alpha) + ((1 - alpha) * AS5600_pos_f64);
	state.vel = (1e6 / TIM5->CNT) * ((uint16_t)AS5600_pos_f64 - state.pos.angle) * AS5600_RAD_CONV;  // rad / s
	state.pos.angle = (uint16_t)AS5600_pos_f64;
	TIM5->CNT = 0;
}  // TODO: integrate vel and acc <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim != &htim10) { return; }
	//if (state.queue_size == 0) { return; }
	euler_method();  // update state.pos.angle, AS5600_delta_pos using the selected mode
	double target_delta = instruction.target - (double)state.pos.angle;

	// TODO: buffer older measurements
	// TODO: create optimal path or remove the 0 to 4096 jump in error when rotating
	// TODO: consider passing rotation dir with spi
	// TODO: add ease in ease out
	// TODO: tune interrupt timing
	// TODO: add the mode switching code back
	// TODO: different filter types: Infinite impulse response (IIR)

	// TODO: add continuous rotation space (-inf, inf) (software)
	// TODO: add defining rotation dir with SPI (this is obviously implicated with the previous TODO)
	// TODO: re-do: ease-in ease-out function
	// TODO: delete / re-do: pathfinding code
	// TODO: tune interrupt timing

	HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, target_delta < 0);
	step_gain = MIN(1, ABS(target_delta / 1024));  // all deltas greater than 1/8 rotation are met by a gain of 100%
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	// only hspi1 is used so there is no need to check
	HAL_GPIO_WritePin(STATUS_PIN_GPIO_Port, STATUS_PIN_Pin, 1);
	if ((instruction_input.crc != crc16_dnp((uint8_t*)&instruction_input, offsetof(MCU_Instruction, crc)))) { set_status(CRC_ERROR); return; }  // reject instruction if crc does not match
	MCU_Instruction* new = malloc(sizeof(MCU_Instruction));
	memcpy(new, &instruction_input, sizeof(MCU_Instruction));
	push(queue, new);  // push to top of queue
	state.queue_size = queue->size;
	HAL_GPIO_WritePin(STATUS_PIN_GPIO_Port, STATUS_PIN_Pin, 0);  // signal to the arm computer that instruction is received correctly
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// this will not instantly load next instruction
	if (GPIO_Pin == INSTUCTION_INT_Pin) {
		pop(queue);
		if (!queue->end) { return; }  // do not update instruction so that the position is held
		memcpy(&instruction, queue->start->data, sizeof(MCU_Instruction));
		set_motor_setting(&instruction);
		step_gain = 0;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// TODO: MAKE SPI INPUT BUFFER like uart buffer !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	status_parity();
	queue = new_list();

	// <TEST>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	instruction.target = 1000;
	instruction.max_vel = 0;
	instruction.max_acc = 0;
	instruction.micro_step = 3;
	instruction.srd_mode = 1;
	instruction.action = 0xf;
	instruction.id = 0;
	instruction.instrution_id = 0;
	// </TEST>

	sensor = AS5600_new();
	sensor->i2c_handle = &hi2c1;
	sensor->dir_port = AS5600_DIR_GPIO_Port;
	sensor->dir_pin = AS5600_DIR_Pin;
	sensor->positive_rotation_direction = AS5600_DIR_CW;
	// TODO: test if hysteresis helps with rotation detection stability
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
	// disable motor driver
	HAL_GPIO_WritePin(STEPPER_NEN_GPIO_Port, STEPPER_NEN_Pin, 1);

	// enable messages
	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&state, (uint8_t*)&instruction_input, 32);

	// initialize AS5600 sensor
	set_status(SENSOR_ERROR);
	HAL_GPIO_WritePin(STATUS_PIN_GPIO_Port, STATUS_PIN_Pin, 1);  // set flag and set error code after first fault
	while (AS5600_init(sensor) != HAL_OK) {}  // the sensor has to be on for the code to work
	HAL_GPIO_WritePin(STATUS_PIN_GPIO_Port, STATUS_PIN_Pin, 0);  // reset flag
	reset_status(SENSOR_ERROR);  // reset error status (so that this isn't read later on)

	state.vel = 0.0;
	state.acc = 0.0;
	state.instrution_id = 0;
	state.queue_size = 0;
	state.micro_step = 0;
	state.srd_mode = 0;
	// initialize the state struct using AS5600 position values
	AS5600_get_angle(sensor, &state.raw_angle);
	state.pos.angle =	state.raw_angle;
	AS5600_pos_f64 =	state.raw_angle;  // set the current angle to the most accurate value for the euler method
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// all DMA connectors are configured in cyclic mode so these functions have to be called once
	/*  DISABLED FOR TESTING
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&instruction, 16);  // start data receiving loop
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&state, 16);  // start data receiving loop
	*/

	// TODO: use flag pin to tell CTRL where MCU is in initialization cycle
	// start communication with CTRL
	HAL_TIM_Base_Start(&htim2);  // start timer_2 (for delays)
	HAL_TIM_Base_Start(&htim5);  // start timer_5 (for simulation time keeping)

	// start receiving ADC data
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&state.raw_angle, 1);

	//instruction.pulse_delay = 74; // 74;  // safe operating range is from 75us and up

	// TODO: Add function to the INSTRUCT_GO interrupt pin that will start the stepping function
	// TODO: FIX ADC NOW IT STARTS FROM 500 AND HANGS ON 4096
	HAL_GPIO_WritePin(STEPPER_NEN_GPIO_Port, STEPPER_NEN_Pin, 0);  // TODO: remove this in final version
	TIM5->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim10);  // start timer_10  (sensor interupt) [100Hz]
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
