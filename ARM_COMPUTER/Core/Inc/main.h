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
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* ACTION_FLAGS
 * part of instruction that tells MCU and CTRL what to do with the provided data
 */
// 0xf
typedef enum {
	EXEC = 0x01,
	OVERRIDE = 0x02,
	SYNC = 0x04,
	POLL = 0x08
} ACTION_FLAGS;

/* State
 * state of the MCU
 */
// 0xffffffffffffffff 0xffffffffffffffff 0xffffffff 0xffffffff 0xffff 0xffff 0xff 0xff 0xff 0xff
typedef struct {  // uint8_t[32]
	double			vel;
	double			acc;
	struct {  // +- 188,744,040 deg
		int32_t		rotation: 20;
		uint32_t	angle: 12;
	}				pos, target;
	uint16_t		raw_angle;
	uint16_t		instrution_id;		// instruction id which is currently being executed
	uint16_t		micro_step: 2;  	// microstep setting
	uint16_t		srd_mode: 1;		// srd mode on the motor controller
	uint16_t		queue_size: 13;		// allows up to 8192 instructions in queue
	// lots of parity for status
	uint16_t		status: 4;			// status codes
	uint16_t		n_status: 4;		// ~status
	uint16_t		status_parity: 1;	// parity for status
	uint16_t		id : 7;				// reserved until the main controller fills this in
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
	uint16_t	action: 4;		// look in ACTION enum for possible actions  // TODO: rethink and implement
	uint16_t	_: 2;
	uint16_t	id: 7;			// selected motor
	uint16_t	instrution_id;	// instruction id
	uint16_t	__;				// reserved uint8_t[2]
	uint16_t	crc;
} MCU_Instruction;

/* Handshake
 * this struct is used to establish a handshake between pc and mcu
 * it deliberatly has no error checking so that unstable connections are not accepted
 * to establish a successfull connection the received data is just sent back to the mcu with some flags set to preform init actions
 */
// 0xffffffff ((0xfe 0x1 0xff) => 0xffff) 0xffff
typedef struct {  // uint8_t[8]
	uint32_t baud;				// TODO: switch baud on the go
	uint16_t motor_count: 7;	// = max motor id (starts counting from 0)
	uint16_t init_0: 1;			// make mcu initialize the motors to their 0 position
	uint16_t _: 8;				// reserved because this struct is minimally 8 bytes
	uint16_t crc;
} CTRL_Handshake;  // control handshake
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t validate_handshake(void*);
extern uint8_t validate_MCU_Instruction(void*);
extern uint8_t get_status(MCU_State* state);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C_INT_Pin GPIO_PIN_12
#define C_INT_GPIO_Port GPIOB
#define CS_0_Pin GPIO_PIN_13
#define CS_0_GPIO_Port GPIOB
#define CF_0_Pin GPIO_PIN_14
#define CF_0_GPIO_Port GPIOB
#define CS_1_Pin GPIO_PIN_15
#define CS_1_GPIO_Port GPIOB
#define CF_1_Pin GPIO_PIN_6
#define CF_1_GPIO_Port GPIOC
#define CS_2_Pin GPIO_PIN_7
#define CS_2_GPIO_Port GPIOC
#define CF_2_Pin GPIO_PIN_8
#define CF_2_GPIO_Port GPIOC
#define CS_3_Pin GPIO_PIN_9
#define CS_3_GPIO_Port GPIOC
#define CF_3_Pin GPIO_PIN_8
#define CF_3_GPIO_Port GPIOA
#define CS_4_Pin GPIO_PIN_9
#define CS_4_GPIO_Port GPIOA
#define CF_4_Pin GPIO_PIN_10
#define CF_4_GPIO_Port GPIOA
#define CS_5_Pin GPIO_PIN_11
#define CS_5_GPIO_Port GPIOA
#define CF_5_Pin GPIO_PIN_12
#define CF_5_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define SYNC_BYTE 0x5C
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
