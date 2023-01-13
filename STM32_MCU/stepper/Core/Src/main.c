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
// min and max measured values
#define MIN_ADC_IN 480
#define MAX_ADC_IN 4000
#define ACD_RANGE_CONV 4096 / (MAX_ADC_IN - MIN_ADC_IN)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MCU_State					state;					// structure that contains the current state of important variables
volatile MCU_Instruction*	instruction = queue;	// pointer to current instruction in queue
volatile MCU_Instruction	queue[MAX_QUEUE_SIZE];	// 1kb (max 256 indexes)
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
void euler_method() {  // typical execution time ~45 us
	register uint16_t raw = ACD_RANGE_CONV * (state.raw_angle - MIN_ADC_IN);
	register uint16_t pos_diff = (AS5600_pos_f64 - raw);  // rotation detection
	state.pos.rotation += pos_diff > 2048; state.pos.rotation -= pos_diff < -2048;
	register double alpha = 1 / ((EULER_TAU / TIM5->CNT) + 1);
	AS5600_pos_f64 = (raw * alpha) + ((1 - alpha) * AS5600_pos_f64);
	state.vel = (1e6 / TIM5->CNT) * ((uint16_t)AS5600_pos_f64 - state.pos.angle) * AS5600_RAD_CONV;  // rad / s
	state.pos.angle = (uint16_t)AS5600_pos_f64;
	TIM5->CNT = 0;
}
void* get_next_empty_queue_ptr() {
	if (state.queue_size == MAX_QUEUE_SIZE) { return 0; }  // nullptr if full
	void* ptr = &queue[(state.queue_index + state.queue_size) % MAX_QUEUE_SIZE];
	return ptr;
}
void* get_next_queue_ptr() {
	if (!state.queue_size) { return 0; }  // nullptr if queue is empty
	state.queue_index = (state.queue_index + 1) % MAX_QUEUE_SIZE;
	state.queue_size--;  // flag last instruction as overwriteable
	return &queue[state.queue_index];
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim != &htim10) { return; }
	if (state.queue_size == 0) { return; }
	//if (pre_euler_func) { (*pre_euler_func)(); }
	euler_method();  // update state.pos.angle, AS5600_delta_pos using the selected mode
	double target_delta = instruction->target - state.pos.angle;
	target_delta = ABS(target_delta) < ABS(target_delta - 4096) ? target_delta : target_delta - 4096;

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

	if (ABS(target_delta) > 10) {  // ~1 deg
		HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, target_delta < 0);
		step_gain = MIN(1, ABS(target_delta / 1024));  // all deltas greater than 1/8 rotation are met by a gain of 100%
		// TODO: change the function
	} else if (!state.lock) {
		instruction = get_next_queue_ptr();  // decrements queue_size and increments queue_index
		if (!instruction) { return; }
		// HAL_GPIO_WritePin(STEPPER_NEN_GPIO_Port, STEPPER_NEN_Pin, !instruction);  // disable stepper when no instruction is loaded
		set_motor_setting(instruction);
		step_gain = 0;
		state.lock = 1;
		// TODO: build variables for stepper function here to call in div above
	}
	return;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	// only hspi1 is used so there is no need to check
	void* dst = get_next_empty_queue_ptr();
	if (!dst) { return; }  // exit if queue is full
	// the main computer is told how its data is recieved via the Status pin
	// note that the pin is cleared when something is received correctly
	// this means that the main computer has to check this pin before sending the next instruction to prevent data loss
	HAL_GPIO_WritePin(STATUS_PIN_GPIO_Port, STATUS_PIN_Pin, \
		(instruction_input.crc != crc16_dnp((uint8_t*)&instruction_input, 30))
	);
	memcpy(dst, (void*)&instruction_input, sizeof(MCU_Instruction));
	state.queue_size++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// this will not instantly load next instruction
	if (GPIO_Pin == NSS_Pin) { HAL_GPIO_WritePin(STATUS_PIN_GPIO_Port, STATUS_PIN_Pin, 1); }  // set the flag pin until reset from SPI_TxRxCplt callback on success
	if (GPIO_Pin == INSTUCTION_INT_Pin) { state.lock = 0; }
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

	// initialize AS5600 sensor
	while (AS5600_init(sensor) != HAL_OK) {}  // the sensor has to be on for the code to work

	state.vel = 0.0;
	state.acc = 0.0;
	state.instrution_id = 0;
	state.queue_size = 0;
	state.queue_index = 31;
	state.micro_step = 0;
	state.srd_mode = 0;
	state.lock = 0;  // TODO: reset this from within the GO_INTERRUPT
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

	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&state, (uint8_t*)&instruction_input, 32);

	// start receiving ADC data
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&state.raw_angle, 1);

	//instruction.pulse_delay = 74; // 74;  // safe operating range is from 75us and up

	// TODO: Add function to the INSTRUCT_GO interrupt pin that will start the stepping function
	// TODO: FIX ADC NOW IT STARTS FROM 500 AND HANGS ON 4096
	HAL_GPIO_WritePin(STEPPER_NEN_GPIO_Port, STEPPER_NEN_Pin, 0);
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
