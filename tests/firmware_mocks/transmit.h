#ifndef TEST_TRANSMIT_H
#define TEST_TRANSMIT_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#define UART_FRAME_SIZE 64
typedef struct { int id; } UART_HandleTypeDef;
typedef struct { int id; } DMA_HandleTypeDef;
typedef void *osMessageQueueId_t;
typedef int BaseType_t;
typedef struct { char command[64]; double params[6]; int param_count; } CommandResult;
extern UART_HandleTypeDef huart1, huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
#define HUART (&huart1)
#define HDMA_HUART_RX (&hdma_usart1_rx)
#define DMA_IT_HT 1
#define pdFALSE 0
#define HAL_OK 0
#define __HAL_DMA_DISABLE_IT(handle, flag) ((void)(handle), (void)(flag))
#define portYIELD_FROM_ISR(wake) ((void)(wake))
int HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, int, int);
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, unsigned);
int xQueueSendToBackFromISR(void *, const void *, BaseType_t *);
void transmit_Init(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
CommandResult parse_command(const char *);
#endif
