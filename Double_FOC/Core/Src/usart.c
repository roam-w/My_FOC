/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <string.h>

// DMA接收相关变量
__IO uint8_t rx_buffer[RX_BUFFER_SIZE];     // DMA接收缓冲区（循环模式）
__IO uint16_t rx_buffer_index = 0;          // 当前接收位置索引
__IO uint16_t rx_received_length = 0;       // 实际接收到的数据长度
__IO bool rx_complete_flag = false;         // 接收完成标志

uint8_t user_rx_buffer[RX_BUFFER_SIZE];     // 用户实际处理的缓冲区，无DMA直接写入

// DMA发送相关变量
__IO bool tx_busy_flag = false;             // 发送忙标志
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */
  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */
  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */
  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  初始化DMA接收
  * @retval None
  */
void UART2_DMA_Init_Receive(void)
{
    // 清除接收缓冲区
    memset((void*)rx_buffer, 0, RX_BUFFER_SIZE);
    rx_buffer_index = 0;
    rx_received_length = 0;
    rx_complete_flag = false;
    
    // 启动DMA接收（循环模式）
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);
    
    // 使能串口空闲中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/**
  * @brief  重新启动DMA接收
  * @retval None
  */
void UART2_DMA_Start_Receive(void)
{
    rx_buffer_index = 0;
    rx_received_length = 0;
    rx_complete_flag = false;
    
    // 重启DMA接收
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);
}

/**
  * @brief  获取接收到的数据长度
  * @retval 接收到的数据字节数
  */
uint16_t UART2_DMA_Get_Received_Length(void)
{
    return rx_received_length;
}

/**
  * @brief  获取接收缓冲区指针
  * @retval 接收缓冲区指针
  */
uint8_t* UART2_DMA_Get_Receive_Buffer(void)
{
    // 原返回：return (uint8_t*)rx_buffer;
    return (uint8_t*)user_rx_buffer; // 改为返回用户缓冲区，数据已无累积
}

/**
  * @brief  清除接收完成标志
  * @retval None
  */
void UART2_DMA_Clear_Receive_Flag(void)
{
    rx_complete_flag = false;
}

/**
  * @brief  通过DMA发送数据
  * @param  data: 要发送的数据指针
  * @param  size: 数据大小
  * @retval true: 发送成功, false: 发送失败（上一次发送未完成）
  */
bool UART2_DMA_Transmit(uint8_t *data, uint16_t size)
{
    if (tx_busy_flag || size == 0 || data == NULL)
    {
        return false;
    }
    
    tx_busy_flag = true;
    
    // 启动DMA发送
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, data, size);
    
    if (status != HAL_OK)
    {
        tx_busy_flag = false;
        return false;
    }
    
    return true;
}

/**
  * @brief  检查DMA发送是否忙
  * @retval true: 忙, false: 空闲
  */
bool UART2_DMA_Is_Transmit_Busy(void)
{
    return tx_busy_flag;
}

/**
  * @brief  等待DMA发送完成
  * @retval None
  */
void UART2_DMA_Wait_Transmit_Complete(void)
{
    while (tx_busy_flag)
    {
        // 可以添加超时机制
        // 这里使用简单的忙等待
    }
}

/**
  * @brief  DMA接收完成回调函数（需要在DMA接收完成中断中调用）
  * @retval None
  */
void UART2_DMA_RX_Complete_Callback(void)
{
    // 对于循环DMA接收，这个函数不会被调用
    // 因为接收模式是DMA_CIRCULAR（循环模式）
}

// 串口回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 确认是USART2的发送完成回调，避免与其他UART冲突
    if (huart->Instance == USART2)
    {
        // 【关键】此时才是真正的发送完成，清零忙标志
        tx_busy_flag = false;
    }
}

/**
  * @brief  串口空闲中断处理函数（需要在USART2_IRQHandler中调用）
  * @retval None
  */
void UART2_IDLE_IRQHandler(void)
{
    // 检查是否是空闲中断
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
    {
        // 1. 清除空闲中断标志（必须手动清除，避免重复触发）
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        
        // 2. 暂停DMA接收（避免拷贝数据期间，DMA继续写入rx_buffer导致数据错乱）
        HAL_UART_DMAStop(&huart2);
        
        // 3. 计算接收到的有效数据长度
        uint16_t remain_data = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
        rx_received_length = RX_BUFFER_SIZE - remain_data;
        
        // 4. 拷贝rx_buffer中的有效数据到用户处理缓冲区（解决累积的核心步骤）
        if (rx_received_length > 0 && rx_received_length <= RX_BUFFER_SIZE)
        {
            // 拷贝有效数据（仅拷贝实际接收的字节数，不拷贝无用残留）
            memcpy(user_rx_buffer, (uint8_t*)rx_buffer, rx_received_length);
            
            // 5. 清空用户缓冲区的剩余部分（彻底解决数据累积，避免下一次数据残留）
            memset(user_rx_buffer + rx_received_length, 0, RX_BUFFER_SIZE - rx_received_length);
        }
        
        // 6. 设置接收完成标志，通知主函数处理用户缓冲区数据
        rx_complete_flag = true;
        
        // 7. 重启DMA循环接收（继续接收下一帧数据，不影响后续传输）
        HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);
        
        // 备注：此时主函数应读取user_rx_buffer中的数据，而非原rx_buffer
    }
}

/* USER CODE END 1 */
