/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {  // MS2, MS1
	RST =	0xfe6f,	// 1111 1111 00 11 1111		bit mask to reset M1 and M2
	M2 =	0x0040,	// 0000 0000 01 00 0000		pin M1 high
	M4 =	0x0080,	// 0000 0000 10 00 0000		pin M2 high
	M8 =	0x0000,	// 0000 0000 00 00 0000		both pins low
	M16 =	0x00b0	// 0000 0000 11 00 0000		both pins high
} MICRO_STEP;

typedef struct {
	int64_t pos;
	int64_t job;
} MCU_State;

typedef struct {
	int64_t steps;
	uint32_t pulse_delay;  // delay from 1us -> 4295s (1 is added to delay because 0 us is not valid)
	struct {
		uint8_t micro_step : 2;  // 0 - 8 (0: MS2, 1: MS4, 2: MS8, 3: MS16)
		uint8_t spread_mode : 1;  // spread mode flag
		uint16_t _ : 13;  // reserved
	} settings;
	uint16_t crc;  // add crc16?
} MCU_Instruction;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MOTOR_STEP_DEG			1.8
#define DEG_TO_M2_STEP_CONV		1 / (MOTOR_STEP_DEG / 2)
#define DEG_TO_M4_STEP_CONV		1 / (MOTOR_STEP_DEG / 4)
#define DEG_TO_M8_STEP_CONV		1 / (MOTOR_STEP_DEG / 8)
#define DEG_TO_M16_STEP_CONV	1 / (MOTOR_STEP_DEG / 16)

#define AS5600_TO_M2_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 2)
#define AS5600_TO_M4_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 4)
#define AS5600_TO_M8_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 8)
#define AS5600_TO_M16_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 16)

/* the tau value is a pre-computed value and describes a system where frequencies of +70KHz are filtered out
 * then multiplied by 10^9 to pre-compute a part of the calculation done with it (define in terms of us))  */
#define EULER_TAU 2.3
// this error margins are applied to the received angle values (in units 360/4096 deg)
#define AS5600_ADC_ERROR_MARGIN 10
#define AS5600_I2C_ERROR_MARGIN 2  /*try changing to 1*/

// structures
extern MCU_State			state;
extern MCU_Instruction		instruction;
extern AS5600_TypeDef*		sensor;
// variables used in the euler method
extern volatile uint16_t	AS5600_analog;		// variable for angles received by ADC (automatic: DMA)
extern uint16_t 			AS5600_i2c;			// variable for angles received by I2C (manual: DMA)
extern volatile uint16_t*	euler_next;			// points to a the variable that will be added using the euler method (either: analog or i2c)
extern double				AS5600_pos_f64;
extern uint16_t				AS5600_pos;
extern int16_t				AS5600_delta_pos;

extern void (*pre_euler_func)(void);

extern double				step_conv;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define MAX(x, y) ((x) > (y)) ? (x) : (y)
#define MIN(x, y) ((x) < (y)) ? (x) : (y)
#define CLAMP(x, y, z) MIN(MAX(x, y), z)
#define ABS_64(x) ((x) > 0) ? (x) : (uint64_t)(x)
#define ABS_16(x) ((x) > 0) ? (x) : (uint16_t)(x)
#define ROUND(x) ((x) > ((int64_t)(x))) ? (((int64_t)(x)) + 1) : ((int64_t)(x))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void i2c_pre_euler(void);
void adc_pre_euler(void);

void delay_us(uint32_t);
void until_us(uint32_t);
void set_motor_setting(MCU_Instruction*);
void euler_method(uint16_t);  // typical execution time ~45 us
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AS5600_ANALOG_IN_Pin GPIO_PIN_0
#define AS5600_ANALOG_IN_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define STEPPER_NEN_Pin GPIO_PIN_12
#define STEPPER_NEN_GPIO_Port GPIOB
#define STEPPER_STP_Pin GPIO_PIN_13
#define STEPPER_STP_GPIO_Port GPIOB
#define STEPPER_DIR_Pin GPIO_PIN_14
#define STEPPER_DIR_GPIO_Port GPIOB
#define STEPPER_SRD_Pin GPIO_PIN_15
#define STEPPER_SRD_GPIO_Port GPIOB
#define STEPPER_MS2_Pin GPIO_PIN_8
#define STEPPER_MS2_GPIO_Port GPIOA
#define STEPPER_MS1_Pin GPIO_PIN_9
#define STEPPER_MS1_GPIO_Port GPIOA
#define AS5600_DIR_Pin GPIO_PIN_5
#define AS5600_DIR_GPIO_Port GPIOB
#define AS5600_SCL_Pin GPIO_PIN_6
#define AS5600_SCL_GPIO_Port GPIOB
#define AS5600_SDA_Pin GPIO_PIN_7
#define AS5600_SDA_GPIO_Port GPIOB
#define __Pin GPIO_PIN_9
#define __GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
