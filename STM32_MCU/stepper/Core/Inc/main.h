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
#include "crc.h"
#include "list.h"
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

typedef enum {
	// OK codes
	OK = 0x0,
	INSTRUCTION_FINISHED = 0x1,
	// soft errors
	INSTRUCTION_UNATTAINABLE = 0x2,
	// hard errors
	CRC_ERROR = 0x4,
	SENSOR_ERROR = 0x8,
} STATUS_CODES;

/* State
 * state of the MCU
 */
// 0xffffffffffffffff 0xffffffffffffffff 0xffffffff 0xffffffff 0xffff 0xffff 0xff 0xff 0xff 0xff
typedef struct {  // uint8_t[32]
	volatile double		vel;
	volatile double		acc;
	volatile struct {  // +- 188,744,040 deg
		int32_t			rotation: 20;
		uint32_t		angle: 12;
	}					pos, target;
	volatile uint16_t	raw_angle;
	uint16_t			instrution_id;		// instruction id which is currently being executed
	uint16_t			micro_step: 2;  	// microstep setting
	uint16_t			srd_mode: 1;		// srd mode on the motor controller
	volatile uint16_t	queue_size: 13;		// allows up to 8192 instructions in queue
	// lots of parity for status
	volatile uint16_t	status: 4;			// status codes
	volatile uint16_t	n_status: 4;		// ~status
	volatile uint16_t	status_parity: 1;	// parity for status
	uint16_t			id : 7;				// TODO: set this by instruction (init to 0x7f)
} MCU_State;


/* Instruction
 * motor instruction (includes flags)
 */
// 0xffffffffffffffff 0xffffffffffffffff 0xffffffffffffffff ((0x3, 0x4, 0x78, 0xff80) => 0xffff) 0xffff, 0xffff, 0xffff
typedef struct {  // uint8_t[32]
	double		target;			// rad
	double		max_vel;		// rad / s
	double		max_acc;		// rad / s^2
	uint16_t	micro_step: 2;  // microstep setting
	uint16_t	srd_mode: 1;	// srd mode on the motor controller
	uint16_t	action: 4;		// look in ACTION enum for possible actions
	uint16_t	dir: 2;			// 0 CLOSEST, 1 CW, 2 CCW, 3 LONGEST
	uint16_t	id: 7;			// selected motor
	uint16_t	instrution_id;	// instruction id
	uint16_t	_;				// reserved uint8_t[2]
	uint16_t	crc;
} MCU_Instruction;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MOTOR_STEP_DEG			1.8
#define DEG_TO_M2_STEP_CONV		1 / (MOTOR_STEP_DEG / 2)
#define DEG_TO_M4_STEP_CONV		1 / (MOTOR_STEP_DEG / 4)
#define DEG_TO_M8_STEP_CONV		1 / (MOTOR_STEP_DEG / 8)
#define DEG_TO_M16_STEP_CONV		1 / (MOTOR_STEP_DEG / 16)

#define AS5600_TO_M2_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 2)
#define AS5600_TO_M4_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 4)
#define AS5600_TO_M8_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 8)
#define AS5600_TO_M16_STEP_CONV	AS5600_DEG_CONV / (MOTOR_STEP_DEG / 16)

#define MAX_QUEUE_SIZE 32

/* the tau value is a pre-computed value and describes a system where frequencies of +70KHz are filtered out
 * then multiplied by 10^9 to pre-compute a part of the calculation done with it (define in terms of us))  */
#define EULER_TAU 2.3
#define MIN_STEPPER_GAIN 1e-5
#define MIN_STEPPER_DELAY 75
// this error margins are applied to the received angle values (in units 360/4096 deg)
#define AS5600_ADC_ERROR_MARGIN 10
#define AS5600_I2C_ERROR_MARGIN 2  /*try changing to 1*/

// structures
extern MCU_State					state;					// structure that contains the current state of important variables
extern volatile MCU_Instruction		instruction; 			// pointer to current instruction in queue
extern volatile Linked_List*		queue;
extern volatile MCU_Instruction		instruction_input;		// instruction directly received from SPI (this will be put into queue if there is space)
extern AS5600_TypeDef*				sensor;

extern volatile double				AS5600_pos_f64; 		// accumulator
extern volatile double				step_gain;				// - (backward) || + (forward) || 0 (idle) [uniform distribution between -1 and 1]

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define MAX(x, y) ((x) > (y)) ? (x) : (y)
#define MIN(x, y) ((x) < (y)) ? (x) : (y)
#define CLAMP(x, y, z) MIN(MAX(x, y), z)
#define ABS(x) ((x) > 0) ? (x) : -(x)
#define ROUND(x) ((x) > ((int64_t)(x))) ? (((int64_t)(x)) + 1) : ((int64_t)(x))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void status_parity(void);
void set_status(uint8_t);
void reset_status(uint8_t);

void delay_us(uint32_t);
void until_us(uint32_t);
void set_motor_setting(MCU_Instruction*);
void euler_method(void);			// typical execution time ~45 us
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AS5600_ANALOG_IN_Pin GPIO_PIN_0
#define AS5600_ANALOG_IN_GPIO_Port GPIOA
#define STATUS_PIN_Pin GPIO_PIN_2
#define STATUS_PIN_GPIO_Port GPIOA
#define INSTUCTION_INT_Pin GPIO_PIN_3
#define INSTUCTION_INT_GPIO_Port GPIOA
#define INSTUCTION_INT_EXTI_IRQn EXTI3_IRQn
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define STEPPER_DIR_Pin GPIO_PIN_12
#define STEPPER_DIR_GPIO_Port GPIOB
#define STEPPER_STP_Pin GPIO_PIN_13
#define STEPPER_STP_GPIO_Port GPIOB
#define STEPPER_SRD_Pin GPIO_PIN_14
#define STEPPER_SRD_GPIO_Port GPIOB
#define STEPPER_MS2_Pin GPIO_PIN_15
#define STEPPER_MS2_GPIO_Port GPIOB
#define STEPPER_MS1_Pin GPIO_PIN_8
#define STEPPER_MS1_GPIO_Port GPIOA
#define STEPPER_NEN_Pin GPIO_PIN_9
#define STEPPER_NEN_GPIO_Port GPIOA
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
