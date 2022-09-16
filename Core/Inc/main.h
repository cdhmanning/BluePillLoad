/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
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
void systick_hook(void);
uint32_t get_tick(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ON_LED_Pin GPIO_PIN_13
#define ON_LED_GPIO_Port GPIOC
#define INC_DEC_LED_Pin GPIO_PIN_14
#define INC_DEC_LED_GPIO_Port GPIOC
#define V_ADC_Pin GPIO_PIN_0
#define V_ADC_GPIO_Port GPIOA
#define I_ADC_Pin GPIO_PIN_2
#define I_ADC_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_6
#define PWM_GPIO_Port GPIOA
#define DEBUG0_Pin GPIO_PIN_0
#define DEBUG0_GPIO_Port GPIOB
#define DEBUG1_Pin GPIO_PIN_1
#define DEBUG1_GPIO_Port GPIOB
#define ROT_INT_Pin GPIO_PIN_12
#define ROT_INT_GPIO_Port GPIOB
#define ROT_INT_EXTI_IRQn EXTI15_10_IRQn
#define ROT_PB_Pin GPIO_PIN_13
#define ROT_PB_GPIO_Port GPIOB
#define ROT_A_Pin GPIO_PIN_14
#define ROT_A_GPIO_Port GPIOB
#define PBUTTON_Pin GPIO_PIN_3
#define PBUTTON_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define ON_LED ON_LED_GPIO_Port, ON_LED_Pin
#define INC_DEC_LED INC_DEC_LED_GPIO_Port, INC_DEC_LED_Pin
#define ROT_A ROT_A_GPIO_Port, ROT_A_Pin
#define ROT_PB  ROT_PB_GPIO_Port, ROT_PB_Pin
#define PBUTTON PBUTTON_GPIO_Port, PBUTTON_Pin
#define DEBUG0 DEBUG0_GPIO_Port, DEBUG0_Pin
#define DEBUG1 DEBUG1_GPIO_Port, DEBUG1_Pin

/* HAL peripheral handles declared in main.c */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
