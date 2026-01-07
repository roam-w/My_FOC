/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define UART_RX_BUFFER_SIZE		256
#define UART_TX_BUFFER_SIZE		256
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
//串口接收回调函数类型
typedef void (*UART_RxCallback)(uint8_t *data, uint16_t length);

//串口DMA管理结构体
typedef struct{
	uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
	uint8_t rx_temp_buffer[UART_RX_BUFFER_SIZE];
	uint16_t rx_data_length;
	volatile bool rx_complete;	//接收完成标志
	bool dma_receiving;					//DMA正在接收标志
	
	uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
	uint16_t tx_data_length;
	volatile bool tx_complete;	//发送完成标志
	bool dma_transmitting;			//DMA正在发送标志
	
	UART_RxCallback rx_callback;		//接收完成回调函数
}UART_DMA_Manager;

//函数声明
// 获取串口管理器
UART_DMA_Manager* UART_GetManager(UART_HandleTypeDef *huart);

void UART_InitDMAReceiver(UART_HandleTypeDef *huart, UART_DMA_Manager *manager);
void UART_StartReceive(UART_HandleTypeDef *huart, UART_DMA_Manager *manager);
bool UART_ReceiveData(UART_HandleTypeDef *huart, UART_DMA_Manager *manager, uint8_t *buffer, uint16_t size);
bool UART_SendData(UART_HandleTypeDef *huart, UART_DMA_Manager *manager, uint8_t *data, uint16_t length);
bool UART_IsTxComplete(UART_DMA_Manager *manager);
void UART_RegisterRxCallback(UART_DMA_Manager *manager, UART_RxCallback callback);
void UART_IDLE_IRQHandler(UART_HandleTypeDef *huart, UART_DMA_Manager *manager);


// 新增辅助函数
void UART_ClearBuffer(UART_DMA_Manager *manager);
bool UART_WaitForData(UART_DMA_Manager *manager, uint32_t timeout);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

