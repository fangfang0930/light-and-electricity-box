/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#define LED1_ON	 			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF	 		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED2_ON	 			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF	 		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED3_ON	 			  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_OFF	 		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED4_ON	 			  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET)
#define LED4_OFF	 		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define LED5_ON	 			  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET)
#define LED5_OFF	 		  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET)
#define LED6_ON	 			  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET)
#define LED6_OFF	 		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET)
#define LED7_ON	 			  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET)
#define LED7_OFF	 		  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET)
#define LED11_ON	 			HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, GPIO_PIN_RESET)
#define LED11_OFF	 		  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, GPIO_PIN_SET)
#define LED12_ON	 			HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, GPIO_PIN_RESET)
#define LED12_OFF	 		  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, GPIO_PIN_SET)
#define LED13_ON	 			HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, GPIO_PIN_RESET)
#define LED13_OFF	 		  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, GPIO_PIN_SET)
#define LED14_ON	 			HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, GPIO_PIN_RESET)
#define LED14_OFF	 		  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, GPIO_PIN_SET)
#define LED15_ON	 			HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_RESET)
#define LED15_OFF	 		  HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_SET)


#define LED_RED_ON	 		HAL_GPIO_WritePin(GPIOI, LED_RED_Pin, GPIO_PIN_RESET)
#define LED_RED_OFF	 		HAL_GPIO_WritePin(GPIOI, LED_RED_Pin, GPIO_PIN_SET)
	
#define LED_GREEN_ON	 	HAL_GPIO_WritePin(GPIOI, LED_GREEN_Pin, GPIO_PIN_RESET)
#define LED_GREEN_OFF	 	HAL_GPIO_WritePin(GPIOI, LED_GREEN_Pin, GPIO_PIN_SET) 
	 
#define LED_BLUE_ON			HAL_GPIO_WritePin(GPIOI, LED_BLUE_Pin, GPIO_PIN_RESET)
#define LED_BLUE_OFF	 	HAL_GPIO_WritePin(GPIOI, LED_BLUE_Pin, GPIO_PIN_SET)

#define TRIP_ON         HAL_GPIO_WritePin(GPIOI, OUT1_Pin, GPIO_PIN_RESET)
#define TRIP_OFF	 	    HAL_GPIO_WritePin(GPIOI, OUT1_Pin, GPIO_PIN_SET)

#define FAULT_ON        HAL_GPIO_WritePin(GPIOI, OUT2_Pin, GPIO_PIN_RESET)
#define FAULT_OFF	 	    HAL_GPIO_WritePin(GPIOI, OUT2_Pin, GPIO_PIN_SET)

#define DEAD_ON         HAL_GPIO_WritePin(GPIOI, OUT3_Pin, GPIO_PIN_RESET)
#define DEAD_OFF	 	    HAL_GPIO_WritePin(GPIOI, OUT3_Pin, GPIO_PIN_SET)

#define IN1_STATE  HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1)
#define IN2_STATE  HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2)
#define IN3_STATE  HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_10)

#define KEY_UP GPIO_PIN_SET
#define KEY_DOWN GPIO_PIN_RESET
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
