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

/* RETURN_CODES
 * message sent back to PC to tell it how instructions are received
 */
// 0x1f
typedef enum {
	RETURN_OK = 0x01,
	RETURN_CRC_FIXED = 0x02,
	RETURN_CRC_ERROR = 0x04,
	RETURN_ERROR_FIXED = 0x08,  // fixed invalid instruction
	RETURN_ERROR = 0x10
} RETURN_FLAGS;
/* Instruction
 * motor instruction (includes flags)
 */
// 0xffffffffffffffff 0xffffffffffffffff 0xffffffffffffffff ((0x3, 0x4, 0x78, 0xff80) => 0xffff) 0xffff
typedef struct {  // uint8_t[28]
	double		target;			// rad
	double		max_vel;		// rad / s
	double		max_acc;		// rad / s^2
	uint16_t	micro_step: 2;  // microstep setting
	uint16_t	srd_mode: 1;	// srd mode on the motor controller
	uint16_t	action: 4;		// look in ACTION enum for possible actions
	uint16_t	id: 9;			// selected motor
	uint16_t	crc;			// TODO (not a priority)
} MCU_Instruction;

/* State
 * state of the motor controller
 */
// 0xffffffff 0xffffffff 0xffffffffffffffffffffffffffffffffffffffff
typedef struct {
	struct {  // +- 188,744,040 deg
		uint32_t sign: 1;
		uint32_t rotation: 19;
		uint32_t angle: 12;
	} pos, target;
	// TODO: add all skewsin variables to state struct
	uint8_t reserved[20];
} MCU_State;

/* Handshake
 * this struct is used to establish a handshake between pc and mcu
 * it deliberatly has no error checking so that unstable connections are not accepted
 * to establish a successfull connection the received data is just sent back to the mcu with some flags set to preform init actions
 */
// ((0x1f, 0x20, 0xffffffc) => 0xffffffff, 0xffff
typedef struct {  // uint8_t[6]
	uint32_t motor_count: 5;	// = max motor id (starts counting from 0)
	uint32_t init_0: 1;			// make mcu initialize the motors to their 0 position
	// set baud rate after handshake
	// first handshake is always done with 9600 baud
	// after change handshake will have to be established again
	// https://community.st.com/s/question/0D53W00001GkPvRSAV/is-it-possible-to-change-the-baud-rate-of-the-usart-during-run-time-in-stm32-cube-ide
	uint32_t baud: 26;
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
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_0_Pin GPIO_PIN_13
#define CS_0_GPIO_Port GPIOB
#define CS_1_Pin GPIO_PIN_14
#define CS_1_GPIO_Port GPIOB
#define CS_2_Pin GPIO_PIN_15
#define CS_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SYNC_BYTE 0x5C
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
