/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define ComRxBufferSize 200
#define ComTxBufferSize 200
/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
typedef struct Struct_ComData
{
		uint16_t    uiRxStartFlag;			 
		uint16_t    uiRxCount;				
		uint16_t    uiTxStartFlag;			 
		uint16_t    uiTxCount;				 
		uint16_t    uiTxCountAll;				
		uint16_t    uiRxReadyFlag;             
		uint16_t    uiRxTimeOut;               
		uint16_t    uiRxTimeOutTh;           
    uint8_t     ucRxBuf[ComRxBufferSize];  
    uint8_t     ucTxBuf[ComTxBufferSize]; 
}StructComData;
extern StructComData ComData3;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern int uart_print(const char * fmt,...);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/