#include "motor.h"

//Motor motor1,motor2,motor3,motor4;
//int START = 0;
//int x_mode = 0;
//extern float TARGET;
//extern float current_yaw;
//extern PID pid_mpu;
//extern float yaw;
//
//#define SPEED_RECORD_NUM 50
//float speed_Record1[SPEED_RECORD_NUM]={0};
//float speed_Record2[SPEED_RECORD_NUM]={0};
//float speed_Record3[SPEED_RECORD_NUM]={0};
//float speed_Record4[SPEED_RECORD_NUM]={0};
//
///*
// * 进行速度的平均滤波
// * 输入新采样到的速度，存放速度的数组，
// * 返回滤波后的速度
// */
//
//void Motor_Clear(){
//    memset(speed_Record1, 0, sizeof(speed_Record1));
//    memset(speed_Record2, 0, sizeof(speed_Record2));
//    memset(speed_Record3, 0, sizeof(speed_Record3));
//    memset(speed_Record4, 0, sizeof(speed_Record4));
//}
//
//float Speed_Low_Filter(float new_Spe,float *speed_Record)
//{
//    float sum = 0.0f;
//    float test_Speed;
//    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
//    {
//        speed_Record[i] = speed_Record[i-1];
//        sum += speed_Record[i-1];
//    }
//    speed_Record[0] = new_Spe;//第一位是新的数据
//    sum += new_Spe;
//    test_Speed = sum/SPEED_RECORD_NUM;
//    return test_Speed;//返回均值
//}
//
///*
// * 清空滤波数据
// * 将速度记录数组中的所有值重置为0
// */
//void Speed_Clear_Low_Filter()
//{
//    for (uint8_t i = 0; i < SPEED_RECORD_NUM; i++) // 遍历数组
//    {
//        speed_Record1[i] = 0.0f; // 将每个元素清零
//        speed_Record2[i] = 0.0f; // 将每个元素清零
//        speed_Record3[i] = 0.0f; // 将每个元素清零
//        speed_Record4[i] = 0.0f; // 将每个元素清零
//    }
//}
//
//
//void Motor_Init()
//{
//	//开启编码器定时器
//    HAL_TIM_Encoder_Start(MOTOR1_ENCODER, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start(MOTOR2_ENCODER, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start(MOTOR3_ENCODER, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start(MOTOR4_ENCODER, TIM_CHANNEL_ALL);
//    //开启编码器定时器更新中断,防溢出处理
//    __HAL_TIM_ENABLE_IT(MOTOR1_ENCODER,TIM_IT_UPDATE);
//    __HAL_TIM_ENABLE_IT(MOTOR2_ENCODER,TIM_IT_UPDATE);
//    __HAL_TIM_ENABLE_IT(MOTOR3_ENCODER,TIM_IT_UPDATE);
//    __HAL_TIM_ENABLE_IT(MOTOR4_ENCODER,TIM_IT_UPDATE);
//    //开启10ms定时器中断
////    HAL_TIM_Base_Start_IT(GAP_TIM);
//    //开启PWM
//    HAL_TIM_PWM_Start(MOTOR1_PWM, MOTOR1_PWM_CHANNEL);
//    HAL_TIM_PWM_Start(MOTOR2_PWM, MOTOR2_PWM_CHANNEL);
//    HAL_TIM_PWM_Start(MOTOR3_PWM, MOTOR3_PWM_CHANNEL);
//    HAL_TIM_PWM_Start(MOTOR4_PWM, MOTOR4_PWM_CHANNEL);
//    //结构体内容初始化
//    motor1.lastCount = 0;
//    motor1.totalCount = 0;
//    motor1.speed = 0;
//    motor1.direction = 0;
//    motor1.in1_GPIO_Port = MOTOR1_IN1_GPIO;
//    motor1.in1_GPIO_Pin = MOTOR1_IN1_PIN;
//    motor1.in2_GPIO_Port = MOTOR1_IN2_GPIO;
//    motor1.in2_GPIO_Pin = MOTOR1_IN2_PIN;
//
//    motor2.lastCount = 0;
//    motor2.totalCount = 0;
//    motor2.speed = 0;
//    motor2.direction = 0;
//    motor2.in1_GPIO_Port = MOTOR2_IN1_GPIO;
//    motor2.in1_GPIO_Pin = MOTOR2_IN1_PIN;
//    motor2.in2_GPIO_Port = MOTOR2_IN2_GPIO;
//    motor2.in2_GPIO_Pin = MOTOR2_IN2_PIN;
//
//    motor3.lastCount = 0;
//    motor3.totalCount = 0;
//    motor3.speed = 0;
//    motor3.direction = 0;
//    motor3.in1_GPIO_Port = MOTOR3_IN1_GPIO;
//    motor3.in1_GPIO_Pin = MOTOR3_IN1_PIN;
//    motor3.in2_GPIO_Port = MOTOR3_IN2_GPIO;
//    motor3.in2_GPIO_Pin = MOTOR3_IN2_PIN;
//
//    motor4.lastCount = 0;
//    motor4.totalCount = 0;
//    motor4.speed = 0;
//    motor4.direction = 0;
//    motor4.in1_GPIO_Port = MOTOR4_IN1_GPIO;
//    motor4.in1_GPIO_Pin = MOTOR4_IN1_PIN;
//    motor4.in2_GPIO_Port = MOTOR4_IN2_GPIO;
//    motor4.in2_GPIO_Pin = MOTOR4_IN2_PIN;
//}
//
//
////电机停止转动
//void Motor_No(Motor *xx){
//	xx->direction = 0;
//	HAL_GPIO_WritePin(xx->in1_GPIO_Port, xx->in1_GPIO_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(xx->in2_GPIO_Port, xx->in2_GPIO_Pin, GPIO_PIN_SET);
//}
//
////电机正转
//void Motor_Plus(Motor *xx){
//	xx->direction = 1;
//	HAL_GPIO_WritePin(xx->in1_GPIO_Port, xx->in1_GPIO_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(xx->in2_GPIO_Port, xx->in2_GPIO_Pin, GPIO_PIN_RESET);
//}
//
////电机反转
//void Motor_Inverse(Motor *xx){
//	xx->direction = -1;
//	HAL_GPIO_WritePin(xx->in1_GPIO_Port, xx->in1_GPIO_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(xx->in2_GPIO_Port, xx->in2_GPIO_Pin, GPIO_PIN_SET);
//}
//
//void Motor_Start(){
//	START = 1;
//}
//
//void Motor_Go(){
//	x_mode = 1;
//	PID_Speed_Target();
//	Motor_Plus(&motor1);
//	Motor_Plus(&motor2);
//	Motor_Plus(&motor3);
//	Motor_Plus(&motor4);
//	pid_mpu.target = current_yaw;
//	yaw = pid_mpu.target;
//	PID_Speed_Target();
//	Motor_Start();
////	vTaskDelay(pdMS_TO_TICKS(1));
//}
//
//void Motor_Back(){
//	x_mode = 1;
//	PID_Speed_Target_Fan();
//	Motor_Inverse(&motor1);
//	Motor_Inverse(&motor2);
//	Motor_Inverse(&motor3);
//	Motor_Inverse(&motor4);
//	pid_mpu.target = current_yaw;
//	yaw = pid_mpu.target;
//	PID_Speed_Target_Fan();
//	Motor_Start();
//}
//
//void Motor_Right(){
//	x_mode = 2;
//	PID_Speed_Target();
//	Motor_Plus(&motor1);
//	Motor_Inverse(&motor2);
//	Motor_Plus(&motor3);
//	Motor_Inverse(&motor4);
//	pid_mpu.target = current_yaw + 90;
//	yaw = current_yaw;
//	Motor_Start();
//}
//
//void Motor_Left(){
//	x_mode = 2;
////	PID_Speed_Zero();
//	PID_Speed_Target();
//	Motor_Inverse(&motor1);
//	Motor_Plus(&motor2);
//	Motor_Inverse(&motor3);
//	Motor_Plus(&motor4);
//	pid_mpu.target = current_yaw - 90;
//	yaw = current_yaw;
//	Motor_Start();
//}
//
//void Motor_Stop(){
//	PID_Speed_Zero();
//	Motor_No(&motor1);
//	Motor_No(&motor2);
//	Motor_No(&motor3);
//	Motor_No(&motor4);
//	START = 0;
//}
//
//
//extern PID pid_speed1;
//extern PID pid_speed2;
//extern PID pid_speed3;
//extern PID pid_speed4;
//extern PID pid_mpu;
//
//extern float target;
//
//
//
////使用pid进行调整
//void Motor_Adjust(){
//	extern float yaw;
//	float data = PID_MPU_Realize(&pid_mpu, yaw);
////	printf("%f,  %f\n",yaw, data);
//
//	pid_speed1.target += data;
//	pid_speed2.target -= data;
//	pid_speed3.target += data;
//	pid_speed4.target -= data;
////	printf("%f,%f,%f,%f\n",pid_speed1.target, pid_speed2.target, pid_speed3.target, pid_speed4.target);
//
//	Limit_Speed(&pid_speed1);
//	Limit_Speed(&pid_speed2);
//	Limit_Speed(&pid_speed3);
//	Limit_Speed(&pid_speed4);
////	printf("speed_target=%f,%f,%f,%f\n", pid_speed1.target, pid_speed2.target, pid_speed3.target, pid_speed4.target);
//
//	Motor_Speed_Adjust();
//}
//
//
////使用pid进行调整
//void Motor_Speed_Adjust()
//{
//	motor1.totalCount = COUNTERNUM1;
//	motor2.totalCount = COUNTERNUM2;
//	motor3.totalCount = COUNTERNUM3;
//	motor4.totalCount = COUNTERNUM4;
//
//	//一个定时器更新时间内的编码器的变化量用dataCount表示
//	//if语句是为了防止越界
//	int32_t dataCount1=0, dataCount2=0, dataCount3=0, dataCount4=0;
//	if(motor1.direction == 1){
//		dataCount1 = motor1.totalCount - motor1.lastCount;
//		if(dataCount1 < -1000) dataCount1 = dataCount1 + 2000;
//	}
//	else if(motor1.direction == -1){
//		dataCount1 = motor1.lastCount-motor1.totalCount;
//		if(dataCount1 < -1000) dataCount1 = dataCount1 + 2000;
//	}
//
//	if(motor2.direction == 1){
//		dataCount2 = motor2.totalCount - motor2.lastCount;
//		if(dataCount2 < -1000) dataCount2 = dataCount2 + 2000;
//	}
//	else if(motor2.direction == -1){
//		dataCount2 = motor2.lastCount-motor2.totalCount;
//		if(dataCount2 < -1000) dataCount2 = dataCount2 + 2000;
//	}
//
//	if(motor3.direction == 1){
//		dataCount3 = motor3.totalCount - motor3.lastCount;
//		if(dataCount3 < -1000) dataCount3 = dataCount3 + 2000;
//	}
//	else if(motor3.direction == -1){
//		dataCount3 = motor3.lastCount-motor3.totalCount;
//		if(dataCount3 < -1000) dataCount3 = dataCount3 + 2000;
//	}
//
//	if(motor4.direction == 1){
//		dataCount4 = motor4.totalCount - motor4.lastCount;
//		if(dataCount4 < -1000) dataCount4 = dataCount4 + 2000;
//	}
//	else if(motor4.direction == -1){
//		dataCount4 = motor4.lastCount-motor4.totalCount;
//		if(dataCount4 < -1000) dataCount4 = dataCount4 + 2000;
//	}
//
////	printf("totalCount=  %d,%d,%d,%d\n",motor1.totalCount,motor2.totalCount,motor3.totalCount,motor4.totalCount);
////	printf("lastCount=  %d,%d,%d,%d\n",motor1.lastCount,motor2.lastCount,motor3.lastCount,motor4.lastCount);
////	printf("dataCount= %d,%d,%d,%d\n",dataCount1,dataCount2,dataCount3,dataCount4);
//
//	motor1.speed = motor1.direction * (float)(dataCount1) / PULSE_PRE_ROUND * 100 * LINE_SPEED_C; //算得车轮线速度每秒多少厘米
//	motor1.lastCount = motor1.totalCount;  //记录这一次的计数值
//	motor1.speed = Speed_Low_Filter(motor1.speed,speed_Record1);
//
//	motor2.speed = motor2.direction * (float)(dataCount2) / PULSE_PRE_ROUND * 100 * LINE_SPEED_C; //算得车轮线速度每秒多少厘米
//	motor2.lastCount = motor2.totalCount;  //记录这一次的计数值
//	motor2.speed = Speed_Low_Filter(motor2.speed,speed_Record2);
//
//	motor3.speed = motor3.direction * (float)(dataCount3) / PULSE_PRE_ROUND * 100 * LINE_SPEED_C; //算得车轮线速度每秒多少厘米
//	motor3.lastCount = motor3.totalCount;  //记录这一次的计数值
//	motor3.speed = Speed_Low_Filter(motor3.speed,speed_Record3);
//
//	motor4.speed = motor4.direction * (float)(dataCount4) / PULSE_PRE_ROUND * 100 * LINE_SPEED_C; //算得车轮线速度每秒多少厘米
//	motor4.lastCount = motor4.totalCount;  //记录这一次的计数值
//	motor4.speed = Speed_Low_Filter(motor4.speed,speed_Record4);
//
//
//	float motor_out1 = PID_Speed_Realize(&pid_speed1,motor1.speed);
//	float motor_out2 = PID_Speed_Realize(&pid_speed2,motor2.speed);
//	float motor_out3 = PID_Speed_Realize(&pid_speed3,motor3.speed);
//	float motor_out4 = PID_Speed_Realize(&pid_speed4,motor4.speed);
////	printf("%f,%f,%f,%f\n",pid_speed1.target, pid_speed2.target, pid_speed3.target, pid_speed4.target);
//	printf("%f,%f,%f,%f\n",motor1.speed,motor2.speed,motor3.speed,motor4.speed);
//	printf("motor_out1= %f,%f,%f,%f\n",motor_out1,motor_out2,motor_out3,motor_out4);
//
//	//起步阶段防止地面摩擦力过大导致的速度骤增，进行平滑过渡
////	float k1 = 0.85 * (1 - cos(M_PI * (fabs(motor1.speed) / (0.2 * fabs(pid_speed1.target))))) + 0.15;
////	pid_speed1.maxOutput = pid_speed1.normalMaxOutput * k1;
////
////	float k2 = 0.85 * (1 - cos(M_PI * (fabs(motor2.speed) / (0.2 * fabs(pid_speed2.target))))) + 0.15;
////	pid_speed2.maxOutput = pid_speed2.normalMaxOutput * k2;
////
////	float k3 = 0.85 * (1 - cos(M_PI * (fabs(motor3.speed) / (0.2 * fabs(pid_speed3.target))))) + 0.15;
////	pid_speed3.maxOutput = pid_speed3.normalMaxOutput * k3;
////
////	float k4 = 0.85 * (1 - cos(M_PI * (fabs(motor4.speed) / (0.2 * fabs(pid_speed4.target))))) + 0.15;
////	pid_speed4.maxOutput = pid_speed4.normalMaxOutput * k4;
//
//
//	extern PID pid_mpu;
//	if(x_mode == 2){
//		if(fabs(pid_mpu.err) < 30.0f){
//			float k;
//			k = 1 / (30.0 - fabs(pid_mpu.err));
//			motor_out1 *= k;
//			motor_out2 *= k;
//			motor_out3 *= k;
//			motor_out4 *= k;
//		}
//
//
//		if(fabs(pid_mpu.err) < 1.0f){
//			motor_out1 = 0;
//			motor_out2 = 0;
//			motor_out3 = 0;
//			motor_out4 = 0;
//			PID_Clear();
//		}
//	}
//
////	printf("motor_out= %f,%f,%f,%f\n",motor_out1,motor_out2,motor_out3,motor_out4);
//
//	if(motor_out1 > 0) Motor_Plus(&motor1);
//	else if(motor_out1 < 0) Motor_Inverse(&motor1);
//
//	if(motor_out2 > 0) Motor_Plus(&motor2);
//	else if(motor_out2 < 0) Motor_Inverse(&motor2);
//
//	if(motor_out3 > 0) Motor_Plus(&motor3);
//	else if(motor_out3 < 0) Motor_Inverse(&motor3);
//
//	if(motor_out4 > 0) Motor_Plus(&motor4);
//	else if(motor_out4 < 0) Motor_Inverse(&motor4);
//
//	int output_min = 60;
//	if(fabs(motor_out1) < output_min) motor_out1 = output_min;
//	if(fabs(motor_out2) < output_min) motor_out2 = output_min;
//	if(fabs(motor_out3) < output_min) motor_out3 = output_min;
//	if(fabs(motor_out4) < output_min) motor_out4 = output_min;
//
//	//调整对应电机的PWM值
//	__HAL_TIM_SetCompare(MOTOR1_PWM, MOTOR1_PWM_CHANNEL, fabs(motor_out1));
//	__HAL_TIM_SetCompare(MOTOR2_PWM, MOTOR2_PWM_CHANNEL, fabs(motor_out2));
//	__HAL_TIM_SetCompare(MOTOR3_PWM, MOTOR3_PWM_CHANNEL, fabs(motor_out3));
//	__HAL_TIM_SetCompare(MOTOR4_PWM, MOTOR4_PWM_CHANNEL, fabs(motor_out4));
//
//}
