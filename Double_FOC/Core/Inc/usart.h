/* USER CODE BEGIN Header */
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
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_SIZE 256    // DMA接收缓冲区大小
#define TX_BUFFER_SIZE 256    // DMA发送缓冲区大小
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
// DMA接收相关函数
void UART2_DMA_Init_Receive(void);
void UART2_DMA_Start_Receive(void);
uint16_t UART2_DMA_Get_Received_Length(void);
uint8_t* UART2_DMA_Get_Receive_Buffer(void);
void UART2_DMA_Clear_Receive_Flag(void);

// DMA发送相关函数
bool UART2_DMA_Transmit(uint8_t *data, uint16_t size);
bool UART2_DMA_Is_Transmit_Busy(void);
void UART2_DMA_Wait_Transmit_Complete(void);

// 中断回调函数（需要在stm32f1xx_it.c中调用）
void UART2_DMA_RX_Complete_Callback(void);
void UART2_DMA_TX_Complete_Callback(void);
void UART2_IDLE_IRQHandler(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

