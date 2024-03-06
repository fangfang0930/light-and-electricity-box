/**
  ******************************************************************************
  * File Name          : FSMC.h
  * Description        : This file provides code for the configuration
  *                      of the FSMC peripheral.
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
#ifndef __FSMC_H
#define __FSMC_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern NOR_HandleTypeDef hnor1;

/* USER CODE BEGIN Private defines */
#define fpga_write(offset,data)	*((volatile unsigned short int *)(0x60000000 + (offset << 1))) = data
#define fpga_read(offset)	*((volatile unsigned short int *)(0x60000000 + (offset << 17)))
//#define fpga_read(offset)	*((volatile unsigned short int *)(0x60000000 + (offset << 1)))
#define FPGA_LED_RED		32
#define FPGA_LED_GREEN  33
#define FPGA_LED_BLUE   34

#define ON  0
#define OFF 1
#define FPGA_LED_RED_ON				fpga_write(FPGA_LED_RED,ON)
#define FPGA_LED_RED_OFF			fpga_write(FPGA_LED_RED,OFF)

#define FPGA_LED_GREEN_ON			fpga_write(FPGA_LED_GREEN,ON)
#define FPGA_LED_GREEN_OFF		fpga_write(FPGA_LED_GREEN,OFF)

#define FPGA_LED_BLUE_ON			fpga_write(FPGA_LED_BLUE,ON)
#define FPGA_LED_BLUE_OFF			fpga_write(FPGA_LED_BLUE,OFF)
/* USER CODE END Private defines */

void MX_FSMC_Init(void);
void HAL_NOR_MspInit(NOR_HandleTypeDef* hnor);
void HAL_NOR_MspDeInit(NOR_HandleTypeDef* hnor);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__FSMC_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
