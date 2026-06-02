/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "event_groups.h"
#include "message_buffer.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STRING_GO "GO"				//指令前进
#define STRING_RIGHT "RIGHT"		//指令右转
#define STRING_LEFT "LEFT"			//指令左转
#define STRING_SERIAL "SERIAL"		//指令控制舵机
#define STRING_MOTOR "MOTOR"		//指令控制电机
#define STRING_UT "UT"				//指令超声�????????????????????????????????????????????????
#define STRING_SPEED "SPEED"		//指令控制车子速度
#define STRING_STOP "STOP"			//指令停止
#define STRING_BACK "BACK"			//指令后�??
#define STRING_ARM "ARM"			//指令机械臂控�??????????
#define STRING_CLAW "CLAW"			//指令机械爪控�??????????
#define STRING_CHANGE "CHANGE"		//指令更换桶子
#define STRING_KP "KP"
#define STRING_KI "KI"
#define STRING_KD "KD"

#define BITMASK_GO 		(0b00000001<<0)
#define BITMASK_RIGHT 	(0b00000001<<1)
#define BITMASK_LEFT 	(0b00000001<<2)
#define BITMASK_SERIAL  (0b00000001<<3)
#define BITMASK_MOTOR   (0b00000001<<4)
#define BITMASK_UT      (0b00000001<<5)
#define BITMASK_BACK    (0b00000001<<6)
#define BITMASK_ARM  	(0b00000001<<7)
#define BITMASK_CLAW	(0b00000001<<8)
#define BITMASK_CHANGE	(0b00000001<<9)

#define MSG_BUFFER_LEN 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float position[3];
extern Motor motors[4];
extern Forward_Ctrl fctrl;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
MessageBufferHandle_t msgbuffer_SERIAL, msgbuffer_MOTOR, msgbuffer_GO, msgbuffer_CLAW, msgbuffer_CHANGE,
		msgbuffer_LEFT, msgbuffer_RIGHT, msgbuffer_UT, msgbuffer_BACK, msgbuffer_ARM;
/* USER CODE END Variables */
/* Definitions for Task_contact */
osThreadId_t Task_contactHandle;
const osThreadAttr_t Task_contact_attributes = {
  .name = "Task_contact",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_go */
osThreadId_t Task_goHandle;
const osThreadAttr_t Task_go_attributes = {
  .name = "Task_go",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_right */
osThreadId_t Task_rightHandle;
const osThreadAttr_t Task_right_attributes = {
  .name = "Task_right",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_left */
osThreadId_t Task_leftHandle;
const osThreadAttr_t Task_left_attributes = {
  .name = "Task_left",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_ut */
osThreadId_t Task_utHandle;
const osThreadAttr_t Task_ut_attributes = {
  .name = "Task_ut",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_back */
osThreadId_t Task_backHandle;
const osThreadAttr_t Task_back_attributes = {
  .name = "Task_back",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_mpu */
osThreadId_t Task_mpuHandle;
const osThreadAttr_t Task_mpu_attributes = {
  .name = "Task_mpu",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_controls */
osThreadId_t Task_controlsHandle;
const osThreadAttr_t Task_controls_attributes = {
  .name = "Task_controls",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_arm */
osThreadId_t Task_armHandle;
const osThreadAttr_t Task_arm_attributes = {
  .name = "Task_arm",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_claw */
osThreadId_t Task_clawHandle;
const osThreadAttr_t Task_claw_attributes = {
  .name = "Task_claw",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_change */
osThreadId_t Task_changeHandle;
const osThreadAttr_t Task_change_attributes = {
  .name = "Task_change",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue_communication */
osMessageQueueId_t Queue_communicationHandle;
const osMessageQueueAttr_t Queue_communication_attributes = {
  .name = "Queue_communication"
};
/* Definitions for motorSemaphores */
osSemaphoreId_t motorSemaphoresHandle;
const osSemaphoreAttr_t motorSemaphores_attributes = {
  .name = "motorSemaphores"
};
/* Definitions for mpuSemaphores */
osSemaphoreId_t mpuSemaphoresHandle;
const osSemaphoreAttr_t mpuSemaphores_attributes = {
  .name = "mpuSemaphores"
};
/* Definitions for controlsSemaphores */
osSemaphoreId_t controlsSemaphoresHandle;
const osSemaphoreAttr_t controlsSemaphores_attributes = {
  .name = "controlsSemaphores"
};
/* Definitions for EventGroup */
osEventFlagsId_t EventGroupHandle;
const osEventFlagsAttr_t EventGroup_attributes = {
  .name = "EventGroup"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int n = 0;
int count = 0;
int last = 0;
double k[3] = { 0.00, 0.00, 0.00 };
char name[3] = "pid";
/* USER CODE END FunctionPrototypes */

void APPTask_contact(void *argument);
void APPTask_go(void *argument);
void APPTask_right(void *argument);
void APPTask_left(void *argument);
void APPTask_ut(void *argument);
void APPTask_back(void *argument);
void APPTask_mpu(void *argument);
void APPTask_controls(void *argument);
void APPTask_arm(void *argument);
void APPTask_claw(void *argument);
void APPTask_change(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of motorSemaphores */
  motorSemaphoresHandle = osSemaphoreNew(1, 1, &motorSemaphores_attributes);

  /* creation of mpuSemaphores */
  mpuSemaphoresHandle = osSemaphoreNew(1, 1, &mpuSemaphores_attributes);

  /* creation of controlsSemaphores */
  controlsSemaphoresHandle = osSemaphoreNew(1, 1, &controlsSemaphores_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_communication */
  Queue_communicationHandle = osMessageQueueNew (36, 51, &Queue_communication_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_contact */
  Task_contactHandle = osThreadNew(APPTask_contact, NULL, &Task_contact_attributes);

  /* creation of Task_go */
  Task_goHandle = osThreadNew(APPTask_go, NULL, &Task_go_attributes);

  /* creation of Task_right */
  Task_rightHandle = osThreadNew(APPTask_right, NULL, &Task_right_attributes);

  /* creation of Task_left */
  Task_leftHandle = osThreadNew(APPTask_left, NULL, &Task_left_attributes);

  /* creation of Task_ut */
  Task_utHandle = osThreadNew(APPTask_ut, NULL, &Task_ut_attributes);

  /* creation of Task_back */
  Task_backHandle = osThreadNew(APPTask_back, NULL, &Task_back_attributes);

  /* creation of Task_mpu */
  Task_mpuHandle = osThreadNew(APPTask_mpu, NULL, &Task_mpu_attributes);

  /* creation of Task_controls */
  Task_controlsHandle = osThreadNew(APPTask_controls, NULL, &Task_controls_attributes);

  /* creation of Task_arm */
  Task_armHandle = osThreadNew(APPTask_arm, NULL, &Task_arm_attributes);

  /* creation of Task_claw */
  Task_clawHandle = osThreadNew(APPTask_claw, NULL, &Task_claw_attributes);

  /* creation of Task_change */
  Task_changeHandle = osThreadNew(APPTask_change, NULL, &Task_change_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	msgbuffer_GO = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_LEFT = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_SERIAL = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_MOTOR = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_RIGHT = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_UT = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_BACK = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_ARM = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_CLAW = xMessageBufferCreate(MSG_BUFFER_LEN);
	msgbuffer_CHANGE = xMessageBufferCreate(MSG_BUFFER_LEN);
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of EventGroup */
  EventGroupHandle = osEventFlagsNew(&EventGroup_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_APPTask_contact */
/**
 * @brief  Function implementing the Task_contact thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_contact */
void APPTask_contact(void *argument)
{
  /* USER CODE BEGIN APPTask_contact */
	//定义value字符串数组来存放由上位机存放的指�????????????????????????????????????????????????????????????????????
	char value[50];
	/* Infinite loop */
	for (;;) {

		//未检测到队列里的数据
		if (xQueueReceive(Queue_communicationHandle, &value, pdMS_TO_TICKS(50)) != pdTRUE){
			continue;
		}
		CommandResult res;
		res = parse_command(value);
//		printf("%s,%f,%f,%f,%f,%f,%f\n",res.command,res.params[0],res.params[1],res.params[2],res.params[3],res.params[4],res.params[5]);

		extern double K[3];
		if(strcmp(res.command, STRING_KP) == 0)
		{
			K[0] = res.params[0];
			Forward_Ctrl_Init(K[0],K[1],K[2]);
			printf("%f,%f,%f\n",K[0],K[1],K[2]);
		}

		if(strcmp(res.command, STRING_KI) == 0)
		{
			K[1] = res.params[0];
			Forward_Ctrl_Init(K[0],K[1],K[2]);
			printf("%f,%f,%f\n",K[0],K[1],K[2]);
		}

		if(strcmp(res.command, STRING_KD) == 0)
		{
			K[2] = res.params[0];
			Forward_Ctrl_Init(K[0],K[1],K[2]);
			printf("%f,%f,%f\n",K[0],K[1],K[2]);
		}

//		如果当前指令是GO
		if (strcmp(res.command, STRING_GO) == 0)
		{
			xEventGroupSetBits(EventGroupHandle, BITMASK_GO);
			xMessageBufferSend(msgbuffer_GO, &res.params[0], sizeof(double), pdMS_TO_TICKS(20));
		}

		//如果指令是RIGHT
		if (strcmp(res.command, STRING_RIGHT) == 0)
		{
			xEventGroupSetBits(EventGroupHandle, BITMASK_RIGHT);
			xMessageBufferSend(msgbuffer_RIGHT, &res.params[0], sizeof(double), pdMS_TO_TICKS(20));
		}

		//如果指令是LEFT
		if (strcmp(res.command, STRING_LEFT) == 0)
		{
			xEventGroupSetBits(EventGroupHandle, BITMASK_LEFT);
			xMessageBufferSend(msgbuffer_LEFT, &res.params[0], sizeof(double), pdMS_TO_TICKS(20));
		}

		//如果指令是BACK
		if (strcmp(res.command, STRING_BACK) == 0)
		{
			xEventGroupSetBits(EventGroupHandle, BITMASK_BACK);
			xMessageBufferSend(msgbuffer_BACK, &res.params[0], sizeof(double), pdMS_TO_TICKS(20));
		}

		//如果是指令是UT
		if (strcmp(res.command, STRING_UT) == 0)
		{
			xEventGroupSetBits(EventGroupHandle, BITMASK_UT);
			xMessageBufferSend(msgbuffer_UT, &res.params[0], sizeof(double), pdMS_TO_TICKS(20));
		}

		//如果指令是STOP
		if (strcmp(res.command, STRING_STOP) == 0)
		{
			Set_StopMode();
		}

		//如果指令是ARM
		if (strcmp(res.command, STRING_ARM) == 0)
		{
			double value[4] = {res.params[0], res.params[1], res.params[2], res.params[3]};
			xEventGroupSetBits(EventGroupHandle, BITMASK_ARM);
			xMessageBufferSend(msgbuffer_ARM, value, sizeof(double) * 4, pdMS_TO_TICKS(100));
		}

		//如果指令是CLAW
		if(strcmp(res.command, STRING_CLAW) == 0)
		{
			xEventGroupSetBits(EventGroupHandle, BITMASK_CLAW);
			xMessageBufferSend(msgbuffer_CLAW, &res.params[0], sizeof(double), pdMS_TO_TICKS(20));
		}

		//如果指令是CHANGE
		if (strcmp(res.command, STRING_CHANGE) == 0)
		{
			xEventGroupSetBits(EventGroupHandle, BITMASK_CHANGE);
			xMessageBufferSend(msgbuffer_CHANGE, &res.params[0], sizeof(double), pdMS_TO_TICKS(20));
		}
//		transmit_Init();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END APPTask_contact */
}

/* USER CODE BEGIN Header_APPTask_go */
/**
 * @brief Function implementing the Task_go thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_go */
void APPTask_go(void *argument)
{
  /* USER CODE BEGIN APPTask_go */
	double value;
//	float coordinate[3];
//	MPU6050_DMP_Get_Date(coordinate);
	/* Infinite loop */
	for (;;) {
		xEventGroupWaitBits(EventGroupHandle, BITMASK_GO,
		pdTRUE, pdTRUE, portMAX_DELAY);
		xMessageBufferReceive(msgbuffer_GO, &value, sizeof(double), pdMS_TO_TICKS(20));

//		printf("value= %f\n",value);
		Set_StraightMode(value, TARGET);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END APPTask_go */
}

/* USER CODE BEGIN Header_APPTask_right */
/**
 * @brief Function implementing the Task_right thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_right */
void APPTask_right(void *argument)
{
  /* USER CODE BEGIN APPTask_right */
	double value;
	/* Infinite loop */
	for (;;) {
		xEventGroupWaitBits(EventGroupHandle, BITMASK_RIGHT,
		pdTRUE, pdTRUE, portMAX_DELAY);
		xMessageBufferReceive(msgbuffer_RIGHT, &value, sizeof(double), pdMS_TO_TICKS(20));

		Set_TurnMode(value);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END APPTask_right */
}

/* USER CODE BEGIN Header_APPTask_left */
/**
 * @brief Function implementing the Task_left thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_left */
void APPTask_left(void *argument)
{
  /* USER CODE BEGIN APPTask_left */
	double value;
	/* Infinite loop */
	for (;;) {
		xEventGroupWaitBits(EventGroupHandle, BITMASK_LEFT,
		pdTRUE, pdTRUE, portMAX_DELAY);
		xMessageBufferReceive(msgbuffer_LEFT, &value, sizeof(double), pdMS_TO_TICKS(20));

		Set_TurnMode(-value);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END APPTask_left */
}

/* USER CODE BEGIN Header_APPTask_ut */
/**
 * @brief Function implementing the Task_ut thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_ut */
void APPTask_ut(void *argument)
{
  /* USER CODE BEGIN APPTask_ut */
	double value;
	/* Infinite loop */
	for (;;) {
		xEventGroupWaitBits(EventGroupHandle, BITMASK_UT,
		pdFALSE, pdTRUE, portMAX_DELAY);
		xMessageBufferReceive(msgbuffer_UT, &value, sizeof(double), pdMS_TO_TICKS(20));

//		if (receive_data[0] == '1') {
//			HAL_TIM_Base_Start(&htim1);
//			HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
//			HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
//			HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
//			vTaskDelay(pdMS_TO_TICKS(1));
//			HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
//		}
//		if (receive_data[0] == '0') {
//			HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_1);
//			HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);
//		}

//		printf("UT %c\n",receive_data[0]);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END APPTask_ut */
}

/* USER CODE BEGIN Header_APPTask_back */
/**
 * @brief Function implementing the Task_back thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_back */
void APPTask_back(void *argument)
{
  /* USER CODE BEGIN APPTask_back */
	double value;
	/* Infinite loop */
	for (;;) {
		xEventGroupWaitBits(EventGroupHandle, BITMASK_BACK,
		pdTRUE, pdTRUE, portMAX_DELAY);
		xMessageBufferReceive(msgbuffer_BACK, &value, sizeof(double), pdMS_TO_TICKS(20));

//		printf("%f\n",value);
		Set_StraightMode(value, -TARGET);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END APPTask_back */
}

/* USER CODE BEGIN Header_APPTask_mpu */
/**
 * @brief Function implementing the Task_mpu thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_mpu */
void APPTask_mpu(void *argument)
{
  /* USER CODE BEGIN APPTask_mpu */

	/* Infinite loop */
	for (;;) {
		if (xSemaphoreTake(mpuSemaphoresHandle,pdMS_TO_TICKS(10)) == pdTRUE) {
			MPU_Read();
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END APPTask_mpu */
}

/* USER CODE BEGIN Header_APPTask_controls */
/**
 * @brief Function implementing the Task_controls thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APPTask_controls */
void APPTask_controls(void *argument)
{
  /* USER CODE BEGIN APPTask_controls */
	/* Infinite loop */
	for (;;) {
		if (xSemaphoreTake(controlsSemaphoresHandle,pdMS_TO_TICKS(10)) == pdTRUE) {;
		    Forward_Control_Update(); // 更新角度�???????????????
			SpeedControl_UpdateAll(); // 更新速度�???????????????

		}
		vTaskDelay(pdMS_TO_TICKS(5));
	}
  /* USER CODE END APPTask_controls */
}

/* USER CODE BEGIN Header_APPTask_arm */
/**
* @brief Function implementing the Task_arm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APPTask_arm */
void APPTask_arm(void *argument)
{
  /* USER CODE BEGIN APPTask_arm */
	double value[4];
  /* Infinite loop */
  for(;;)
  {
	  xEventGroupWaitBits(EventGroupHandle, BITMASK_ARM, pdTRUE, pdTRUE, portMAX_DELAY);
	  xMessageBufferReceive(msgbuffer_ARM, value, sizeof(double) * 4, pdMS_TO_TICKS(100));
	  printf("[接收成功] Values: %.2f, %.2f, %.2f, %.2f\n",value[0], value[1], value[2], value[3]);
	  PCA_Servo_270(0, value[0]);
	  PCA_Servo_180(1, value[1]);
	  PCA_Servo_180(2, value[2]);
	  PCA_Servo_180(3, value[3]);
	  vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END APPTask_arm */
}

/* USER CODE BEGIN Header_APPTask_claw */
/**
* @brief Function implementing the Task_claw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APPTask_claw */
void APPTask_claw(void *argument)
{
  /* USER CODE BEGIN APPTask_claw */
	double value;
  /* Infinite loop */
  for(;;)
  {
	  xEventGroupWaitBits(EventGroupHandle, BITMASK_CLAW, pdTRUE, pdTRUE, portMAX_DELAY);
	  xMessageBufferReceive(msgbuffer_CLAW, &value, sizeof(double), pdMS_TO_TICKS(20));
//	  printf("claw ok  :%d\n",(int)value);
	  if(value == 1){
		  PCA_Servo_180(5, 90);
	  }
	  else if(value == 0){
		  PCA_Servo_180(5, 0);
	  }
	  vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END APPTask_claw */
}

/* USER CODE BEGIN Header_APPTask_change */
/**
* @brief Function implementing the Task_change thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APPTask_change */
void APPTask_change(void *argument)
{
  /* USER CODE BEGIN APPTask_change */
	double value;
  /* Infinite loop */
  for(;;)
  {
	  xEventGroupWaitBits(EventGroupHandle, BITMASK_CHANGE, pdTRUE, pdTRUE, portMAX_DELAY);
	  xMessageBufferReceive(msgbuffer_CHANGE, &value, sizeof(double), pdMS_TO_TICKS(20));
	  if(value == 0)		//RIGHT
	  {
		  get_out_right();
		  get_in_right();
	  }
	  else if(value == 1)	//LEFT
	  {
		  get_out_left();
		  get_in_left();
	  }
	  else if(value == 2)
	  {
		  get_in_right();
		  get_in_left();
	  }
	  vTaskDelay(pdMS_TO_TICKS(20));
  }
  /* USER CODE END APPTask_change */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
int upEdge = 0;
int downEdge = 0;
int cha = 0;
double distance = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
//		upEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//		downEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
////		printf("up= %d,down= %d\n",upEdge,downEdge);
//		cha = downEdge - upEdge;
//		if (cha < 0)
//			cha += 10000;
//		distance = cha * 0.034 / 2;
////		printf("up=%d,down=%d,cha=%d,dis=%f\n",upEdge,downEdge,cha,distance);
//		printf("%f\n", distance);
//	}
}
/* USER CODE END Application */

