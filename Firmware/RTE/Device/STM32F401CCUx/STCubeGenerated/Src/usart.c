/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "cmsis_os2.h"

/* Private define ------------------------------------------------------------*/
#define UART_EVENT_FLAG_TX_COMPLETED 	(1<<0)
#define UART_EVENT_FLAG_RX_COMPLETED 	(1<<1)

#define MAX_UART_WAIT_DURATION					1000

/* Private variables ---------------------------------------------------------*/
osEventFlagsId_t		EVT_UART6;

/* Private function prototypes -----------------------------------------------*/
static void UART_WaitSending(UART_HandleTypeDef* uartHandle);
static void UART_WaitReceiving(UART_HandleTypeDef* uartHandle);

uint8_t UART_DebugTX_Buff[UART_DEBUG_BUFF_SIZE];
uint8_t UART_DebugRX_Buff[UART_DEBUG_BUFF_SIZE];
/* USER CODE END 0 */

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
	EVT_UART6 = osEventFlagsNew(NULL);
  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PA11     ------> USART6_TX
    PA12     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = UART_DEBUG_TX_Pin|UART_DEBUG_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PA11     ------> USART6_TX
    PA12     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOA, UART_DEBUG_TX_Pin|UART_DEBUG_RX_Pin);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void UART_WaitSending(UART_HandleTypeDef* uartHandle)
{
	osEventFlagsId_t evtFlag = (uartHandle->Instance == USART6)?EVT_UART6:NULL;
	
	if(evtFlag != NULL)
	{
		osEventFlagsWait(evtFlag, UART_EVENT_FLAG_TX_COMPLETED, osFlagsWaitAll, MAX_UART_WAIT_DURATION);
	}
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void UART_WaitReceiving(UART_HandleTypeDef* uartHandle)
{
	osEventFlagsId_t evtFlag = (uartHandle->Instance == USART6)?EVT_UART6:NULL;

	if(evtFlag != NULL)
	{
		osEventFlagsWait(evtFlag, UART_EVENT_FLAG_RX_COMPLETED, osFlagsWaitAll, MAX_UART_WAIT_DURATION);
	}
}

/* Exported functions --------------------------------------------------------*/

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief				Tx Transfer completed callbacks.
	* @note					
	*	@param[IN]  	huart  Pointer to a UART_HandleTypeDef structure that contains
  * 									   the configuration information for the specified UART module.
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	osEventFlagsId_t evtFlag = (huart->Instance == USART6)?EVT_UART6:NULL;
	
	if(evtFlag != NULL)
	{
		osEventFlagsSet(evtFlag, UART_EVENT_FLAG_TX_COMPLETED);
	}
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief				Rx Transfer completed callbacks.
	* @note					
	*	@param[IN]  	huart  Pointer to a UART_HandleTypeDef structure that contains
  *                	 		 the configuration information for the specified UART module.
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	osEventFlagsId_t evtFlag = (huart->Instance == USART6)?EVT_UART6:NULL;
	
	if(evtFlag != NULL)
	{
		osEventFlagsSet(evtFlag, UART_EVENT_FLAG_RX_COMPLETED);
	}
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void UART_AsyncTransmit(UART_HandleTypeDef *huart, uint8_t * data, uint16_t size)
{
	HAL_UART_Transmit_DMA(huart, data, size);
	UART_WaitSending(huart);
}

/**--------------------------------------------------------------------------------------------------------------------------------------------------------------
  * @brief
	* @note				
	*	@param[IN]  	
	*	@param[OUT]		
  * @retval 		
  *--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void UART_AsyncReceive(UART_HandleTypeDef *huart, uint8_t * data, uint16_t size)
{
	HAL_UART_Receive_DMA(huart, data, size);
	UART_WaitReceiving(huart);
}

/* USER CODE END 1 */
