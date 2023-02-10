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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Test_LED_Pin GPIO_PIN_13
#define Test_LED_GPIO_Port GPIOC
#define Led___Buzzer_Pin GPIO_PIN_0
#define Led___Buzzer_GPIO_Port GPIOA
#define Servo_Motor_Pin GPIO_PIN_1
#define Servo_Motor_GPIO_Port GPIOA
#define IR_sensor_Pin GPIO_PIN_0
#define IR_sensor_GPIO_Port GPIOB
#define Push_Button_Pin GPIO_PIN_1
#define Push_Button_GPIO_Port GPIOB
#define Push_Button_EXTI_IRQn EXTI1_IRQn
#define Ultrasonic_Echo_Pin GPIO_PIN_8
#define Ultrasonic_Echo_GPIO_Port GPIOA
#define UART_Transmitter_Pin GPIO_PIN_9
#define UART_Transmitter_GPIO_Port GPIOA
#define UART_Receiver_Pin GPIO_PIN_10
#define UART_Receiver_GPIO_Port GPIOA
#define Ultrasonic_Trig_Pin GPIO_PIN_11
#define Ultrasonic_Trig_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
