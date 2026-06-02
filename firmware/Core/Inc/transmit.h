#ifndef INC_TRANSMIT_H_
#define INC_TRANSMIT_H_

#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "usart.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ------------------------------
// 命令解析结果结构体
// ------------------------------
typedef struct {
    char command[64];     // 存储命令词（如"set_speed"）
    double params[6];      // 存储最多6个参数，索引0~5
    int param_count;      // 实际解析到的参数个数（0~6）
} CommandResult;




#define HUART &huart1
#define HDMA_HUART_RX &hdma_usart1_rx
extern DMA_HandleTypeDef hdma_usart1_rx;

void transmit_Init();
int _write(int file, char *data, int len);
CommandResult parse_command(const char *input);

#endif /* INC_TRANSMIT_H_ */
