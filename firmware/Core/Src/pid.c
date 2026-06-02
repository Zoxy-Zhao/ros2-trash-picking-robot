#include "pid.h"

//PID pid_speed1, pid_speed2, pid_speed3, pid_speed4, pid_mpu;
//float TARGET = 100;
//float target =  0;
//double K[3] = {1.66,0.0,0.0};
//extern int x_mode;
//
//void PID_Set_Target(float a){
//	TARGET = a;
//}
//
//void PID_Set_Mpu_Target(float a){
//	pid_mpu.target = a;
//}
//
//void PID_Speed_Target(){
//	target = TARGET;
//	pid_speed1.target = target;
//	pid_speed2.target = target;
//	pid_speed3.target = target;
//	pid_speed4.target = target;
//}
//
//void PID_Speed_Target_Fan(){
//	target = -TARGET;
//	pid_speed1.target = target;
//	pid_speed2.target = target;
//	pid_speed3.target = target;
//	pid_speed4.target = target;
//}
//
//void PID_Speed_Zero(){
//	target = 0;
//	pid_speed1.target = target;
//	pid_speed2.target = target;
//	pid_speed3.target = target;
//	pid_speed4.target = target;
//}
//
////PID参数初始化
//void PID_Init(void)
//{
//    pid_speed1.err = 0;
//    pid_speed1.integral = 0;
//    pid_speed1.maxIntegral = 1000;
//    pid_speed1.maxOutput = __HAL_TIM_GetAutoreload(MOTOR1_PWM);
//    pid_speed1.normalMaxOutput = pid_speed1.maxOutput;
//    pid_speed1.lastErr = 0;
//    pid_speed1.output = 0;
//    pid_speed1.target = TARGET;
//    pid_speed1.kp = 2.75;
//    pid_speed1.ki = 0.11;
//    pid_speed1.kd = 23;
//
//    pid_speed2.err = 0;
//    pid_speed2.integral = 0;
//    pid_speed2.maxIntegral = 1000;
//    pid_speed2.maxOutput = __HAL_TIM_GetAutoreload(MOTOR2_PWM);
//    pid_speed2.normalMaxOutput = pid_speed2.maxOutput;
//    pid_speed2.lastErr = 0;
//    pid_speed2.output = 0;
//    pid_speed2.target = TARGET;
//    pid_speed2.kp = 2.75;
//    pid_speed2.ki = 0.11;
//    pid_speed2.kd = 23;
//
//    pid_speed3.err = 0;
//    pid_speed3.integral = 0;
//    pid_speed3.maxIntegral = 1000;
//    pid_speed3.maxOutput = __HAL_TIM_GetAutoreload(MOTOR3_PWM);
//    pid_speed3.normalMaxOutput = pid_speed3.maxOutput;
//    pid_speed3.lastErr = 0;
//    pid_speed3.output = 0;
//    pid_speed3.target = TARGET;
//    pid_speed3.kp = 2.75;
//    pid_speed3.ki = 0.11;
//    pid_speed3.kd = 23;
//
//    pid_speed4.err = 0;
//    pid_speed4.integral = 0;
//    pid_speed4.maxIntegral = 1000;
//    pid_speed4.maxOutput = __HAL_TIM_GetAutoreload(MOTOR4_PWM);
//    pid_speed4.normalMaxOutput = pid_speed4.maxOutput;
//    pid_speed4.lastErr = 0;
//    pid_speed4.output = 0;
//    pid_speed4.target = TARGET;
//    pid_speed4.kp = 2.75;
//    pid_speed4.ki = 0.11;
//    pid_speed4.kd = 23;
//
//    pid_mpu.err = 0;
//    pid_mpu.integral = 0;
//    pid_mpu.maxIntegral = 20;
//    pid_mpu.maxOutput = 20;
//    pid_mpu.normalMaxOutput = pid_mpu.maxOutput;
//    pid_mpu.lastErr = 0;
//    pid_mpu.output = 0;
//    pid_mpu.target = 0;
//    extern double K[3];
//    pid_mpu.kp = K[0];
//    pid_mpu.ki = K[1];
//    pid_mpu.kd = K[2];
//}
//
//
////一次速度PID计算
//float PID_Speed_Realize(PID* pid,float feedback)
//{
//	int x = 0;
//	if(pid->target < 0){
//		pid->target = -pid->target;
//		feedback = -feedback;
//		x = 1;
//	}
//    pid->err = pid->target - feedback;
//    if(pid->err < 0.3 && pid->err > -0.3) pid->err = 0;//pid死区
//    pid->integral += pid->err;
//
//    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
//    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;
//
//    if(pid->target == 0) pid->integral = 0; // 刹车时清空i
//
//
//    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID
////    printf("%f\n",pid->output);
//
//    //输出限幅
//
//	if(pid->output < 0) pid->output = 0;
//	else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
//
//    pid->lastErr = pid->err;
//    if(pid->target == 0) pid->output = 0; // 刹车时直接输出0
//    if(x) {
//    	pid->output = -pid->output;
//    	pid->target = -pid->target;
//    }
//    return pid->output;
//}
//
//#define MPU_RECORD_NUM 33  // 假设数组的长度为 10
//float Mpu_Record[MPU_RECORD_NUM] = {0};
//// 低通滤波函数
//float Mpu_Low_Filter(float new_Mpu, float *mpu_Record)
//{
//    float sum = 0.0f;
//    float test_Mpu;
//
//    // 将现有数据后移一位
//    for(uint8_t i = MPU_RECORD_NUM - 1; i > 0; i--)
//    {
//        mpu_Record[i] = mpu_Record[i - 1];  // 将数据后移
//        sum += mpu_Record[i - 1];           // 累加旧数据的值
//    }
//
//    mpu_Record[0] = new_Mpu;  // 第一位是新的数据
//    sum += new_Mpu;  // 加上新数据到总和中
//
//    // 计算均值
//    test_Mpu = sum / MPU_RECORD_NUM;
//
//    return test_Mpu;  // 返回均值
//}
//
//void Mpu_Clear_Low_Filter(float *mpu_Record)
//{
//    // 清空滤波数据
//    for (uint8_t i = 0; i < MPU_RECORD_NUM; i++)
//    {
//        mpu_Record[i] = 0.0f;  // 将每个元素重置为零
//    }
//}
//
//
//extern int x_turn;
//float PID_MPU_Realize(PID* pid, float feedback){
////	printf("%f\n",feedback);
//	if(feedback >= 0) feedback += 0.005;
//	else feedback -= 0.005;
//	int temp = feedback * 100;
//	feedback = temp / 100.0;
////	printf("b= %f\n",feedback);
////	feedback = Mpu_Low_Filter(feedback, Mpu_Record);
////	printf("target= %f, feedback= %f\n",pid_mpu.target, feedback);
//    pid->err = pid->target - feedback;
//    printf("err= %f\n",pid->err);
//    if(pid->err < 0.05 && pid->err > -0.05) pid->err = 0;//pid死区
//
//    pid->integral += pid->err;
//
//    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
//    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;
//    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID
////    printf("feedback= %f,output= %f\n",feedback, pid->output);
////    printf("%f,\n",pid->output);
//    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
//    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
//    pid->lastErr = pid->err;
////    printf("output= %f\n",pid->output);
//    return pid->output;
//}
//
//// 限制电机目标速度
//
//void Limit_Speed(PID* pid_speed) {
//	if(x_mode == 1){
//		int value = 20;
//		if (pid_speed->target > target + value) pid_speed->target = target + value;
//		else if (pid_speed->target < target - value) pid_speed->target = target - value;
//	}
//    if (pid_speed->target > 400) pid_speed->target = 400;
//    else if (pid_speed->target < -400) pid_speed->target = -400;
//}
//
//void PID_Clear(){
//    pid_speed1.integral = 0;
//    pid_speed2.integral = 0;
//    pid_speed3.integral = 0;
//    pid_speed4.integral = 0;
//    pid_mpu.integral = 0;
//}
