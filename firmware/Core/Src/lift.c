#include "lift.h"
#include "pca9685.h"

uint16_t duty = 0;

volatile uint32_t pulse_count = 0;   // 已发送脉冲计 ????
uint32_t target_pulses = 3200;       // 目标脉冲数（1600 ????/转）
uint16_t pulse_freq = 200;

uint8_t is_moving = 0;


void Direction_Servo_ctl(uint16_t duty_1)
{
	duty = (10*(float)duty_1/ 270 +2.5) / 100 * 2000;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,duty);
}
void claw_Servo_ctl(uint16_t duty_1)
{
	duty = (10*(float)duty_1/ 180 +2.5) / 100 * 2000;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,duty);
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM1 && is_moving) {
//        pulse_count++;
//        if (pulse_count >= target_pulses) {
//            // 达到目标脉冲数，停止运动
//            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//            is_moving = 0;
//            pulse_count = 0;
//        }
//    }
//}
// 启动运动（设置方向和目标脉冲数）
void Start_Motion(uint8_t direction, uint32_t pulses) {
    // 设置方向
    HAL_GPIO_WritePin(GPIO_PORT, DIR_PIN, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    // 设置目标脉冲 ????
    target_pulses = pulses;
    pulse_count = 0;

    // 启动PWM
    is_moving = 1;
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

// 立即停止
void Emergency_Stop(void) {
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    is_moving = 0;
    pulse_count = 0;
}
void servo_mid()
{
//	Direction_Servo_ctl(150);
	PCA_Servo_270(6, 150);
}
void servo_left()
{
//	Direction_Servo_ctl(9);
	PCA_Servo_270(6, 9);
}
void servo_right()
{
//	Direction_Servo_ctl(280);
	PCA_Servo_270(6, 289);
}
void claw_Servo_off(){
//	claw_Servo_ctl(166);
	PCA_Servo_180(7, 169);
}
void claw_Servo_on(){
//	claw_Servo_ctl(130);
	PCA_Servo_180(7, 130);
}
void stepper_down()
{
	Start_Motion(1, 2710);
}
void stepper_down_half_right()
{
	Start_Motion(1, 2150);
}
void stepper_down_half_left()
{
	Start_Motion(1, 2100);
}
void stepper_up_half_right()
{
	Start_Motion(0, 2150);
}
void stepper_up_half_left()
{
	Start_Motion(0, 2100);
}
void stepper_up()
{
	Start_Motion(0, 2710);
}

void lift_Init()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);

//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
	Emergency_Stop();
	//Start_Motion(1, 2850);  // 方向1=逆时针，目标2850 ????
	servo_mid(); //方向舵机归中
	claw_Servo_on();
}

void get_in_right()
{
	 claw_Servo_off();
	 HAL_Delay(1000);
	 stepper_up();
	 HAL_Delay(8160);
	 servo_right();
	 HAL_Delay(1000);
	 stepper_down_half_right();
	 HAL_Delay(8160);
	 claw_Servo_on();
	 HAL_Delay(1000);
	 stepper_up_half_right();
	 HAL_Delay(8160);
	 servo_mid();
	 HAL_Delay(1000);
	 stepper_down();
	 HAL_Delay(8160);
}
void get_out_right()
{
	 stepper_up();
	 HAL_Delay(8160);
	 servo_right();
	 HAL_Delay(1000);
	 stepper_down_half_left();
	 HAL_Delay(8160);
	 claw_Servo_off();
	 HAL_Delay(1000);
	 stepper_up_half_left();
	 HAL_Delay(8160);
	 servo_mid();
	 HAL_Delay(1000);
	 stepper_down();
	 HAL_Delay(8160);
	 claw_Servo_on();
	 HAL_Delay(5000);
}
void get_in_left()
{
	 claw_Servo_off();
	 HAL_Delay(1000);
	 stepper_up();
	 HAL_Delay(8160);
	 servo_left();
	 HAL_Delay(1000);
	 stepper_down_half_left();
	 HAL_Delay(8160);
	 claw_Servo_on();
	 HAL_Delay(1000);
	 stepper_up_half_left();
	 HAL_Delay(8160);
	 servo_mid();
	 HAL_Delay(1000);
	 stepper_down();
	 HAL_Delay(8160);
}
void get_out_left()
{
	 stepper_up();
	 HAL_Delay(8160);
	 servo_left();
	 HAL_Delay(1000);
	 stepper_down_half_left();
	 HAL_Delay(8160);
	 claw_Servo_off();
	 HAL_Delay(1000);
	 stepper_up_half_left();
	 HAL_Delay(8160);
	 servo_mid();
	 HAL_Delay(1000);
	 stepper_down();
	 HAL_Delay(8160);
	 claw_Servo_on();
	 HAL_Delay(5000);
}
