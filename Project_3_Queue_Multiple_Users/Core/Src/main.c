/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Thread and Queue handles */
osThreadId_t Sender1Handle;
osThreadId_t Sender2Handle;
osThreadId_t ReceiverHandle;
osMessageQueueId_t myQueue01Handle;

/* Thread attributes */
const osThreadAttr_t Sender1_attributes = {.name = "Sender1", .stack_size = 128*4, .priority = osPriorityNormal};
const osThreadAttr_t Sender2_attributes = {.name = "Sender2", .stack_size = 128*4, .priority = osPriorityNormal};
const osThreadAttr_t Receiver_attributes = {.name = "Receiver", .stack_size = 128*4, .priority = osPriorityNormal};

/* Queue attributes */
const osMessageQueueAttr_t myQueue01_attributes = {.name = "myQueue01"};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void StartSender1(void *argument);
void StartSender2(void *argument);
void StartReceiver(void *argument);

/* USER CODE BEGIN 0 */
/* Retarget printf to UART */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 10);
    return len;
}
/* USER CODE END 0 */

int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    /* Init scheduler */
    osKernelInitialize();

    /* Create message queue */
    myQueue01Handle = osMessageQueueNew(256, sizeof(uint8_t), &myQueue01_attributes);

    /* Create threads */
    Sender1Handle = osThreadNew(StartSender1, NULL, &Sender1_attributes);
    Sender2Handle = osThreadNew(StartSender2, NULL, &Sender2_attributes);
    ReceiverHandle = osThreadNew(StartReceiver, NULL, &Receiver_attributes);

    /* Start scheduler */
    osKernelStart();

    /* Infinite loop */
    while (1) {}
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

/* USART2 init */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 38400;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* GPIO init */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* Sender1 task */
void StartSender1(void *argument)
{
    uint8_t msg = 1;
    for(;;)
    {
        printf("Task1 sending %d\n", msg);
        osMessageQueuePut(myQueue01Handle, &msg, 0, osWaitForever);
        osDelay(2000);
    }
}

/* Sender2 task */
void StartSender2(void *argument)
{
    uint8_t msg = 2;
    for(;;)
    {
        printf("Task2 sending %d\n", msg);
        osMessageQueuePut(myQueue01Handle, &msg, 0, osWaitForever);
        osDelay(2500);
    }
}

/* Receiver task */
void StartReceiver(void *argument)
{
    uint8_t recvMsg;
    for(;;)
    {
        if (osMessageQueueGet(myQueue01Handle, &recvMsg, NULL, osWaitForever) == osOK)
        {
            printf("Receiver got: %d\n", recvMsg);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
    }
}

/* Error handler */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Wrong parameters value: file %s on line %d\n", file, line);
}
#endif
