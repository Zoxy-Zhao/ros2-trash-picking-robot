#ifndef INC_NEW_CONTROLS_H_
#define INC_NEW_CONTROLS_H_

#include "main.h"

#define SPEED_RECORD_NUM 50 	// 平均滤波数组的长度
#define STARTUP_DUTY_GO 120		// 直线启动初始占空比
#define STARTUP_DUTY_TURN 120	// 转向启动初始占空比
#define STARTUP_MS 500      	// 开环阶段持续时间（单位ms）
#define TARGET 25				// 默认速度

// 速度距离限制模块
#define MAX_SPEED_DELTA  20.0f  	// 速度ΔV最大调整量（根据实测调整）
#define SPEED_LIMIT_HIGH  60.0f 	// 最高允许速度
#define SPEED_LIMIT_LOW  10.0f 	    // 最低允许速度
#define TURN_SPEED_LIMIT_HIGH 22.0f	// 最高转向速度
#define TURN_SPEED_LIMIT_LOW 12.5f	// 最低转向速度
#define INFINITY 999				// 无距离限制直线行驶

// 电机1-4的编码器编号
#define MOTOR1_ENCODER &htim2
#define MOTOR2_ENCODER &htim3
#define MOTOR3_ENCODER &htim4
#define MOTOR4_ENCODER &htim5

// 电机的定时器编号
#define MOTOR1_PWM &htim8
#define MOTOR2_PWM &htim8
#define MOTOR3_PWM &htim8
#define MOTOR4_PWM &htim8
#define MOTOR1_PWM_CHANNEL TIM_CHANNEL_1
#define MOTOR2_PWM_CHANNEL TIM_CHANNEL_2
#define MOTOR3_PWM_CHANNEL TIM_CHANNEL_3
#define MOTOR4_PWM_CHANNEL TIM_CHANNEL_4

// 间隔定时器编号
#define GAP_TIM &htim1

#define MAX_PWM	1000								// 定时器的最大PWM值
#define PULSE_PRE_ROUND 1320 						// 一圈多少个脉冲
#define RADIUS_OF_TYRE 3.5							// 轮胎半径，单位厘米
#define RELOADVALUE 2000							// 编码器的重装载值

//获取编码器定时器中的计数值
#define COUNTERNUM1 __HAL_TIM_GetCounter(MOTOR1_ENCODER)
#define COUNTERNUM2 __HAL_TIM_GetCounter(MOTOR2_ENCODER)
#define COUNTERNUM3 __HAL_TIM_GetCounter(MOTOR3_ENCODER)
#define COUNTERNUM4 __HAL_TIM_GetCounter(MOTOR4_ENCODER)

//电机的两个in口
#define MOTOR1_IN1_GPIO IN1_A_GPIO_Port
#define MOTOR1_IN1_PIN IN1_A_Pin
#define MOTOR1_IN2_GPIO IN1_B_GPIO_Port
#define MOTOR1_IN2_PIN IN1_B_Pin

#define MOTOR2_IN1_GPIO IN2_A_GPIO_Port
#define MOTOR2_IN1_PIN IN2_A_Pin
#define MOTOR2_IN2_GPIO IN2_B_GPIO_Port
#define MOTOR2_IN2_PIN IN2_B_Pin

#define MOTOR3_IN1_GPIO IN3_A_GPIO_Port
#define MOTOR3_IN1_PIN IN3_A_Pin
#define MOTOR3_IN2_GPIO IN3_B_GPIO_Port
#define MOTOR3_IN2_PIN IN3_B_Pin

#define MOTOR4_IN1_GPIO IN4_A_GPIO_Port
#define MOTOR4_IN1_PIN IN4_A_Pin
#define MOTOR4_IN2_GPIO IN4_B_GPIO_Port
#define MOTOR4_IN2_PIN IN4_B_Pin

// PID结构体直接嵌入电机控制中
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_err;
    float output;
    float out_max;
    float out_min;
} PID;

typedef struct {
    // 须绑定的硬件资源
    TIM_HandleTypeDef* encoder_tim;   // 编码器定时器（如MOTOR1_ENCODER）
    TIM_HandleTypeDef* pwm_tim;       // PWM定时器（如MOTOR1_PWM）
    uint32_t pwm_channel;             // PWM通道(TIM_CHANNEL_1等)
    GPIO_TypeDef *IN1_GPIO, *IN2_GPIO;// 方向引脚GPIO
    uint16_t IN1_PIN, IN2_PIN;        // 方向引脚PIN

    // 运动控制参数
    int32_t last_counter;        			// 上一次编码器计数值
    float current_speed;         			// 当前速度(cm/s)
    float last_speed;						// 上一次的速度(cm/s)
    float target_speed;          	 		// 目标速度(cm/s)
    PID speed_pid;               			// 速度环控制器
    uint8_t is_right;            			// 标记是否属于右侧电机，便于差速控制
    float filtered_speed;					// 经滤波过滤后的速度
    float speed_Record[SPEED_RECORD_NUM];   // 记录数据的数组
    uint8_t is_starting;    				// 启动状态标志
} Motor;

typedef enum {
	MODE_STRAIGHT,
	MODE_TURN,
    MODE_STOP
} Mode;

// 前向控制全局状态
typedef struct {
    float target_yaw;     			// 目标航向角（陀螺仪yaw）
    Mode mode;			  			// 模式切换
    PID angle_pid;        			// 角度环控制器
    // 距离控制相关
    float target_distance;        	// 总目标距离（cm）
    float accumulated_dist;      	// 已行驶距离（cm）
    float start_base_speed;      	// 初始基准速度（cm/s）
    float brake_zone_length;     	// 减速区长度（基于基速和制动加速度计算）
    uint8_t enable_distance_ctrl; 	// 距离控制使能标志（1为启用）
    uint32_t start_tick;			// 启动时刻
} Forward_Ctrl;


// 初始化函数
void PID_Init(PID *pid, float kp, float ki, float kd, float min, float max);
void Motors_Init(void);
void Forward_Ctrl_Init(float kp, float ki, float kd);
void Motor_SetPWM(Motor* m, float out);

// 核心控制函数
void SpeedControl_UpdateAll(void);  						// 启用速度环函数
void Forward_Control_Update(void);  						// 启用角度环函数
void Set_StopMode();										// 切换停止模式
void Set_StraightMode(float target_cm, float base_speed); 	// 切换直线模式
void Set_TurnMode(float target_angle);						// 切换转向模式




#endif /* INC_NEW_CONTROLS_H_ */
