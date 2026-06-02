#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"


////电机1-4的编码器号
//#define MOTOR1_ENCODER &htim2
//#define MOTOR2_ENCODER &htim3
//#define MOTOR3_ENCODER &htim4
//#define MOTOR4_ENCODER &htim5
//
////电机的定时器号
//#define MOTOR1_PWM &htim8
//#define MOTOR2_PWM &htim8
//#define MOTOR3_PWM &htim8
//#define MOTOR4_PWM &htim8
//#define MOTOR1_PWM_CHANNEL TIM_CHANNEL_1
//#define MOTOR2_PWM_CHANNEL TIM_CHANNEL_2
//#define MOTOR3_PWM_CHANNEL TIM_CHANNEL_3
//#define MOTOR4_PWM_CHANNEL TIM_CHANNEL_4
//
////间隔定时器号
//#define GAP_TIM &htim1
//
//
//#define PULSE_PRE_ROUND 410 //一圈多少个脉冲
//#define RADIUS_OF_TYRE 6.5 //轮胎半径，单位厘米
//#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14
//#define RELOADVALUE 2000
//
////获取编码器定时器中的计数值
//#define COUNTERNUM1 __HAL_TIM_GetCounter(MOTOR1_ENCODER)
//#define COUNTERNUM2 __HAL_TIM_GetCounter(MOTOR2_ENCODER)
//#define COUNTERNUM3 __HAL_TIM_GetCounter(MOTOR3_ENCODER)
//#define COUNTERNUM4 __HAL_TIM_GetCounter(MOTOR4_ENCODER)
//
////电机的两个in口
//#define MOTOR1_IN1_GPIO IN1_A_GPIO_Port
//#define MOTOR1_IN1_PIN IN1_A_Pin
//#define MOTOR1_IN2_GPIO IN1_B_GPIO_Port
//#define MOTOR1_IN2_PIN IN1_B_Pin
//
//#define MOTOR2_IN1_GPIO IN2_A_GPIO_Port
//#define MOTOR2_IN1_PIN IN2_A_Pin
//#define MOTOR2_IN2_GPIO IN2_B_GPIO_Port
//#define MOTOR2_IN2_PIN IN2_B_Pin
//
//#define MOTOR3_IN1_GPIO IN3_A_GPIO_Port
//#define MOTOR3_IN1_PIN IN3_A_Pin
//#define MOTOR3_IN2_GPIO IN3_B_GPIO_Port
//#define MOTOR3_IN2_PIN IN3_B_Pin
//
//#define MOTOR4_IN1_GPIO IN4_A_GPIO_Port
//#define MOTOR4_IN1_PIN IN4_A_Pin
//#define MOTOR4_IN2_GPIO IN4_B_GPIO_Port
//#define MOTOR4_IN2_PIN IN4_B_Pin
//
//
//typedef struct _Motor
//{
//    int32_t lastCount;   //上一次计数值
//    int32_t totalCount;  //总计数值
//    float speed;         //电机转速
//    int direction;  	 //转动方向
//	GPIO_TypeDef *in1_GPIO_Port;
//	uint16_t in1_GPIO_Pin;
//	GPIO_TypeDef *in2_GPIO_Port;
//	uint16_t in2_GPIO_Pin;
//}Motor;
//
//void Motor_Init();
//void Motor_No(Motor *xx);
//void Motor_Plus(Motor *xx);
//void Motor_Inverse(Motor *xx);
//void Motor_Start();
//void Motor_Go();
//void Motor_Back();
//void Motor_Right();
//void Motor_Left();
//void Motor_Stop();
//void Motor_Clear();
//float Speed_Low_Filter(float new_Spe,float *speed_Record);
//void Speed_Clear_Low_Filter();
//void Motor_Adjust();
//void Motor_Speed_Adjust();
#endif /* INC_MOTOR_H_ */
