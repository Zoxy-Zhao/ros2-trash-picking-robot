#include "transmit.h"


uint8_t receiveData[50];
char message[10];
extern DMA_HandleTypeDef hdma_usart2_rx;
int x = 0;

//打印函数
int _write(int file, char *data, int len) {
    int i = 0;
    for(i = 0; i < len; i++) {
        // 发送一个字符到USART1
        HAL_UART_Transmit(&huart2, (unsigned char*)&data[i], 1, portMAX_DELAY);
    }
    return len;
}



//初始化串口
void transmit_Init(){
	//开启接收不定长度数据,由上位机通过串口发送指令
	HAL_UARTEx_ReceiveToIdle_DMA(HUART, (uint8_t *)receiveData, sizeof(receiveData));
	//关闭DMA传输过半中断
	__HAL_DMA_DISABLE_IT(HDMA_HUART_RX, DMA_IT_HT);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)receiveData, sizeof(receiveData));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	printf("1\n");
}


extern osMessageQueueId_t Queue_communicationHandle;

//接受不定长度数据完毕回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == HUART){
		//将由上位机传来的指令传入消息队列中
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		BaseType_t xStatus = xQueueSendToBackFromISR(Queue_communicationHandle, receiveData, xHigherPriorityTaskWoken);
//		if(xHigherPriorityTaskWoken == pdFALSE) printf("pdFALSE\n");
		//队列有空余位置时
		if(xStatus == pdTRUE){
			printf("YES\n");
		}
		//队列已满时
		else printf("NO\n");

		memset(receiveData, 0, sizeof(receiveData));
		//再次开启接收不定长度数据
		HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)receiveData, sizeof(receiveData));
		//关闭DMA传输过半中断
		__HAL_DMA_DISABLE_IT(HDMA_HUART_RX,DMA_IT_HT);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);



//		int nums = -1;
//		for(int i = 0; receiveData[i] != '\0'; i++){
//			nums++;
//		}
//		extern Motor motors[4];
//		extern double K[3];
////		printf("%d\n",nums);
//		if(receiveData[0] == 'K'){
//
//			if(receiveData[1] == 'P'){
//				x = 0;
//			}
//			else if(receiveData[1] == 'I'){
//				x = 1;
//			}
//			else if(receiveData[1] == 'D'){
//				x = 2;
//			}
//				int k = 0;
//				double a = 0;
//				for(int i = 3; i < nums; i++){
//					if(receiveData[i] == '.'){
//						k = 1;
//					}
//					else{
//						a*= 10;
//						k*= 10;
//						a+= receiveData[i] - '0';
//					}
//				}
//				while(k > 1){
//					k/= 10;
//					a/= 10;
//				}
//				if(receiveData[3] == '-'){
//					K[x] = a;
//				}
//				else K[x] = a;
//				if(K[x] < 0) K[x] = 0;
//				printf("%f, %f, %f\n",K[0], K[1], K[2]);
//			for(int i = 0; i < 4; i++){
//				Motor* m = &motors[i];
//				PID_Init(&m->speed_pid, K[0], K[1], K[2], -MAX_PWM, MAX_PWM);
//				if(i == 0) printf("%f,%f,%f\n",m->speed_pid.Kp,m->speed_pid.Ki,m->speed_pid.Kd);
//			}
//		}
//			else {
//				int b = 0, x2 = 0;
//				for(int i = 0; i < nums; i++){
//					if(receiveData[i] == '-') {
//						x2 = 1;
//						continue;
//					}
//					b *= 10;
//					b += receiveData[i] - '0';
//				}
//				if(x2) b = -b;
//				for(int i = 0; i < 4; i++){
//					Motor* m = &motors[i];
//					m->target_speed = b;
//				}
//			}
//
//		memset(receiveData, 0, sizeof(receiveData));
//				//再次开启接收不定长度数据
//				HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)receiveData, sizeof(receiveData));
//				//关闭DMA传输过半中断
//				__HAL_DMA_DISABLE_IT(HDMA_HUART_RX,DMA_IT_HT);


	}

	if (huart == &huart2) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		BaseType_t xStatus = xQueueSendToBackFromISR(Queue_communicationHandle, (char *)receiveData, xHigherPriorityTaskWoken);
		//队列有空余位置时
		if(xStatus == pdTRUE){
			printf("YES\n");
		}
		//队列已满时
		else printf("NO\n");

		HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)receiveData, sizeof(receiveData));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		memset(receiveData, 0, sizeof(receiveData));
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


// =============== 解析命令函数 ===============








// ------------------------------
// 工具函数（支持去除换行符）
// ------------------------------
static void trim_whitespace(char *str) {
    // 去除头尾所有空白字符（含空格、制表符、换行、回车）
    if (!str) return;
    int len = strlen(str);

    // 去除尾部空白
    while (len > 0 && isspace((unsigned char)str[len-1])) {
        str[--len] = '\0';
    }

    // 去除头部空白
    char *start = str;
    while (isspace((unsigned char)*start)) start++;
    if (start != str) {
        memmove(str, start, len - (start - str) + 1);
    }
}

static int is_numeric(const char *str) {
    if (*str == '-' || *str == '+') str++; // 允许符号

    int has_dot = 0;
    while (*str) {
        if (*str == '.' && !has_dot) {
            has_dot = 1;
        } else if (!isdigit(*str)) {
            return 0; // 非数字字符
        }
        str++;
    }
    return 1;
}

// ------------------------------
// 主解析函数（支持6参数）
// ------------------------------
CommandResult parse_command(const char *input) {
    CommandResult res = {0};
    res.param_count = 0;

    // 1. 安全拷贝输入并清理（关键：防止溢出）
    char buffer[256];
    strncpy(buffer, input, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // 强制终止符
    trim_whitespace(buffer);  // 清理首尾空白

    // 2. 分割命令词和参数（支持空格、TAB、换行作为分隔符）
    const char *delim = " \t\r\n";
    char *token = strtok(buffer, delim);

    if (token == NULL) return res; // 处理空输入

    // 3. 保存命令词（直接使用首个token）
    strncpy(res.command, token, sizeof(res.command) - 1);
    res.command[sizeof(res.command) - 1] = '\0'; // 确保终止

    // 4. 循环提取参数（最多6个）
    for (int arg = 0; arg < 6; arg++) {
        token = strtok(NULL, delim);  // 继续分割后续内容

        if (token == NULL) break;     // 无更多参数时退出

        // 清理单个参数内的残留空白（如" 123 "→"123"）
        trim_whitespace(token);

        // 分析参数合法性并存储
        if (is_numeric(token)) {
            res.params[arg] = atof(token);
        } else {
            res.params[arg] = 0.0f; // 非法参数默认0
        }

        res.param_count++; // 增加参数计数
    }

    return res;
}

