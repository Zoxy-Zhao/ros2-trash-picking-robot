#include "new-controls.h"

double K[3] = {0, 0, 0};
double K1[3] = {8.0, 0.25, 37.5};
double K2[3] = {5.6, 0.015, 6.2};

Motor motors[4] = { // [右前, 左前, 右后, 左后]
    // Motor1 (右前)
    {
        .encoder_tim       = MOTOR1_ENCODER,
        .pwm_tim           = MOTOR1_PWM,
        .pwm_channel       = MOTOR1_PWM_CHANNEL,
        .IN1_GPIO          = MOTOR1_IN1_GPIO,
        .IN1_PIN           = MOTOR1_IN1_PIN,
        .IN2_GPIO          = MOTOR1_IN2_GPIO,
        .IN2_PIN           = MOTOR1_IN2_PIN,
        .is_right          = 1,              // 右侧电机
        .speed_pid         = {0}             // 后续在Init中配置
    },
    // Motor2 (左前)
    {
        .encoder_tim       = MOTOR2_ENCODER,
        .pwm_tim           = MOTOR2_PWM,
        .pwm_channel       = MOTOR2_PWM_CHANNEL,
        .IN1_GPIO          = MOTOR2_IN1_GPIO,
        .IN1_PIN           = MOTOR2_IN1_PIN,
        .IN2_GPIO          = MOTOR2_IN2_GPIO,
        .IN2_PIN           = MOTOR2_IN2_PIN,
        .is_right          = 0,              // 左侧电机
        .speed_pid         = {0}
    },
    // Motor3 (右后)
    {
        .encoder_tim       = MOTOR3_ENCODER,
        .pwm_tim           = MOTOR3_PWM,
        .pwm_channel       = MOTOR3_PWM_CHANNEL,
        .IN1_GPIO          = MOTOR3_IN1_GPIO,
        .IN1_PIN           = MOTOR3_IN1_PIN,
        .IN2_GPIO          = MOTOR3_IN2_GPIO,
        .IN2_PIN           = MOTOR3_IN2_PIN,
        .is_right          = 1,
        .speed_pid         = {0}
    },
    // Motor4 (左后)
    {
        .encoder_tim       = MOTOR4_ENCODER,
        .pwm_tim           = MOTOR4_PWM,
        .pwm_channel       = MOTOR4_PWM_CHANNEL,
        .IN1_GPIO          = MOTOR4_IN1_GPIO,
        .IN1_PIN           = MOTOR4_IN1_PIN,
        .IN2_GPIO          = MOTOR4_IN2_GPIO,
        .IN2_PIN           = MOTOR4_IN2_PIN,
        .is_right          = 0,
        .speed_pid         = {0}
    }
};

// PID初始化函数
void PID_Init(PID *pid, float kp, float ki, float kd, float min, float max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0;
    pid->prev_err = 0;
    pid->output = 0;
    pid->out_min = min;
    pid->out_max = max;
}


// 电机批量初始化
void Motors_Init() {
    for (int i = 0; i < 4; i++) {
		Motor* m = &motors[i];
		// 初始化GPIO方向控制（刹车）
		HAL_GPIO_WritePin(m->IN1_GPIO, m->IN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m->IN2_GPIO, m->IN2_PIN, GPIO_PIN_RESET);

		// 初始化编码器
		__HAL_TIM_SET_COUNTER(m->encoder_tim, 0);  // 计数器归零
		m->last_counter = 0;
		HAL_TIM_Encoder_Start(m->encoder_tim, TIM_CHANNEL_ALL);  // 启动编码器模式

		// 初始化PWM通道
		HAL_TIM_PWM_Start(m->pwm_tim, m->pwm_channel);  // 启动指定PWM通道
		__HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_channel, 0);  // PWM占空比初始化为0

		// 初始化速度环PID
		PID_Init(&m->speed_pid, K1[0], K1[1], K1[2], -MAX_PWM, MAX_PWM);
		m->target_speed = 0;

		// 初始化数组
		memset(m->speed_Record, 0, sizeof(m->speed_Record));
    }
    // 启动间隔定时器
    HAL_TIM_Base_Start_IT(GAP_TIM);
}

// 急停函数
static void scram(){
	for(int i=0; i<4; i++){
        Motor* m = &motors[i];
        HAL_GPIO_WritePin(m->IN1_GPIO, m->IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->IN2_GPIO, m->IN2_PIN, GPIO_PIN_SET);
	}
}


Forward_Ctrl fctrl; // 全局前向控制器

// ================================= PID核心算法 =================================
void PID_Update(PID* pid, float target, float current) {
    float err = target - current;

    // 比例项
    float P = pid->Kp * err;

    // 积分项（带抗饱和）
    pid->integral += err;
    if (pid->integral > pid->out_max/pid->Ki)
        pid->integral = pid->out_max/pid->Ki;
    else if (pid->integral < pid->out_min/pid->Ki)
        pid->integral = pid->out_min/pid->Ki;
    float I = pid->Ki * pid->integral;

    // 微分项
    float D = pid->Kd * (err - pid->prev_err);
    pid->prev_err = err;

    // 总和并限幅
    pid->output = P + I + D;
    if (pid->output > pid->out_max)
        pid->output = pid->out_max;
    else if (pid->output < pid->out_min)
        pid->output = pid->out_min;
}

// ================================= 速度环执行 =================================
void Motor_SetPWM(Motor* m, float out) {
    // 方向判断与GPIO设置
    if (out > 0) { // 正转
        HAL_GPIO_WritePin(m->IN1_GPIO, m->IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->IN2_GPIO, m->IN2_PIN, GPIO_PIN_RESET);
    } else {       // 反转
        HAL_GPIO_WritePin(m->IN1_GPIO, m->IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->IN2_GPIO, m->IN2_PIN, GPIO_PIN_SET);
    }
    // PWM占空比设置（取绝对值）
    __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_channel, fabsf(out));
}

// 进行速度的平均滤波
float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    float test_Speed;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//第一位是新的数据
    sum += new_Spe;
    test_Speed = sum/SPEED_RECORD_NUM;
    return test_Speed;//返回均值
}

// 读取速度函数
static float Get_Speed(Motor* m) {
    int32_t curr_cnt = __HAL_TIM_GET_COUNTER(m->encoder_tim);

    // 1. 处理编码器溢出
    int32_t max_count = m->encoder_tim->Init.Period;
    int32_t delta = curr_cnt - m->last_counter;
    if (delta > max_count / 2) delta -= max_count;    // 正向溢出
    else if (delta < -max_count / 2) delta += max_count; // 反向溢出

    m->last_counter = curr_cnt;

    // 2. 速度计算（未滤波）
    float raw_speed = (delta / (float)PULSE_PRE_ROUND) * (2 * 3.1416 * RADIUS_OF_TYRE) / 0.01;

    // 3. 对速度进行平均滤波
    m->filtered_speed = Speed_Low_Filter(raw_speed, m->speed_Record);

//    // —— 4. 死区处理（减少微小波动干扰）——
//    #define DEAD_ZONE 0.5f // cm/s以下视为零漂
//    if(fabs(m->filtered_speed - m->last_speed) < DEAD_ZONE) m->filtered_speed = m->last_speed;
    m->last_speed = m->filtered_speed;

    // ====== 位移累计（单位：cm）======
    float delta_dist = (delta / (float)PULSE_PRE_ROUND) * (2 * 3.1416 * RADIUS_OF_TYRE);

    // 累计总位移（仅主驱动轮参与统计，假设前轮为驱动轮）
    if(m == &motors[0] || m == &motors[1]) {
        fctrl.accumulated_dist += fabs(delta_dist) * 0.5; // 取前两轮的平均值
    }
//    printf("fctrl.accumulated_dist==%f\n",fctrl.accumulated_dist);


    return m->filtered_speed;
}


// 启用速度环函数
void SpeedControl_UpdateAll(void) {
    for (int i = 0; i < 4; i++) {
        Motor* m = &motors[i];
        // ==== 启动阶段处理 ====
        if(m->is_starting == 1) {
//        	printf("启动阶段\n");
        	if(fctrl.mode == MODE_STRAIGHT){
        		if(fabs(m->speed_pid.output) < STARTUP_DUTY_GO){
//                if(fabs(m->current_speed) < 0.25 * fabs(m->target_speed)) {
                    float start_duty = (m->target_speed > 0) ? STARTUP_DUTY_GO : -STARTUP_DUTY_GO;
                    m->current_speed = Get_Speed(m);
                    PID_Update(&m->speed_pid, m->target_speed, m->current_speed);
                    Motor_SetPWM(m, start_duty); // 强制定向推力
//                    printf("%f\n",start_duty);
                    continue; // 跳过PID
                }
                else {
                	// 退出启动状态
                	motors[0].is_starting = 0;
                	motors[1].is_starting = 0;
                	motors[2].is_starting = 0;
                	motors[3].is_starting = 0;
                }
        	}
        	if(fctrl.mode == MODE_TURN){
        		if(fabs(m->speed_pid.output) < STARTUP_DUTY_TURN){
//        		if(fabs(m->current_speed) < 0.25 * fabs(m->target_speed)) {
        			m->current_speed = Get_Speed(m);
        			PID_Update(&m->speed_pid, m->target_speed, m->current_speed);
                	int dir = (fctrl.target_yaw < Gyro_GetYaw()) ? 1 : -1;
					if(motors[i].is_right) {
						Motor_SetPWM(&motors[i], -STARTUP_DUTY_TURN * dir);
					} else {
						Motor_SetPWM(&motors[i], STARTUP_DUTY_TURN * dir);
					}
                    continue; // 跳过PID
                }
                else {
                	// 退出启动状态
                	motors[0].is_starting = 0;
                	motors[1].is_starting = 0;
                	motors[2].is_starting = 0;
                	motors[3].is_starting = 0;
                }
        	}
        }

        // ==== 正常PID控制流程 ====
		m->current_speed = Get_Speed(m);
		if(m->current_speed == 0 && m->target_speed == 0)
			motors[i].speed_pid.integral = 0.0f;
		PID_Update(&m->speed_pid, m->target_speed, m->current_speed);
		Motor_SetPWM(m, m->speed_pid.output);
    }
//    printf("%f,%f\n",motors[0].current_speed,motors[0].target_speed);
//    printf("%d  %d  %d  %d\n",__HAL_TIM_GET_COUNTER(motors[0].encoder_tim), __HAL_TIM_GET_COUNTER(motors[1].encoder_tim), __HAL_TIM_GET_COUNTER(motors[2].encoder_tim), __HAL_TIM_GET_COUNTER(motors[3].encoder_tim));
//    printf("%f,%f,%f,%f,%f\n",motors[0].target_speed,motors[0].current_speed,motors[1].current_speed,motors[2].current_speed,motors[3].current_speed);
//    printf("%f,%f,%f,%f\n",motors[0].speed_pid.output,motors[1].speed_pid.output,motors[2].speed_pid.output,motors[3].speed_pid.output);
}
// ================================= 角度环控制 =================================
void Forward_Ctrl_Init(float kp, float ki, float kd) {
    PID_Init(&fctrl.angle_pid, kp, ki, kd, -50, 50); // ΔV限幅±50cm/s
    fctrl.mode = MODE_STOP;
}

// 切换停止模式
void Set_StopMode(){
	fctrl.mode = MODE_STOP;
	for(int i=0; i<4; i++){
		motors[i].is_starting = 0;
	}
}


// 切换直线模式(target_cm表示需要前进或后退的距离，可正可负
//           base_speed表示前进或后退时的速度，为正值。)
void Set_StraightMode(float target_cm, float base_speed) {
// 1. 初始化直线模式（保持航向）
	// 设置当前方向为目标方向
	fctrl.target_yaw = Gyro_GetYaw();
	// 重置角度环PID积分
	fctrl.angle_pid.integral = 0;
	fctrl.angle_pid.prev_err = 0;

// 2. 设置距离控制相关参数
    float dir = (target_cm >= 0) ? 1.0 : -1.0;
    if(fabs(target_cm) == INFINITY) {
    	fctrl.enable_distance_ctrl = 0; // 无限距离模式
    	fctrl.start_base_speed = dir * base_speed;
    }

    else if(fabs(target_cm) > 0.001f) {
        fctrl.enable_distance_ctrl = 1;
        fctrl.target_distance = fabs(target_cm);
        fctrl.brake_zone_length = 10.0f;
        fctrl.start_base_speed = dir * base_speed;
        fctrl.accumulated_dist = 0.0f;
    }

// 3. 统一设置电机基准速度
    for(int i=0; i<4; i++) {
    	memset(motors[i].speed_Record, 0, sizeof(motors[i].speed_Record));
        motors[i].target_speed = dir * base_speed;
//        printf("%f\n",motors[i].target_speed);
    }

// 4. 启动模式初始化
    for(int i=0; i<4; i++){
        motors[i].is_starting = 1;  // 进入启动阶段
    }
    fctrl.start_tick = HAL_GetTick();  // 记录启动时刻

// 5.设置直线模式
    fctrl.mode = MODE_STRAIGHT;
}

// 切换转向模式(target_angle表示顺时针旋转的角度)
void Set_TurnMode(float target_angle){
// 1. 初始化转向模式
	// 设置转向角度
	float current_yaw = Gyro_GetYaw();
	fctrl.target_yaw = current_yaw - (target_angle * 18 / 19);
	// 重置角度环PID积分
	fctrl.angle_pid.integral = 0;
	fctrl.angle_pid.prev_err = 0;
// 2. 统一设置电机基准速度为0
	for(int i=0; i<4; i++){
    	memset(motors[i].speed_Record, 0, sizeof(motors[i].speed_Record));
    	motors[i].target_speed = 0.0f;
	}

// 3. 启动模式初始化
    for(int i=0; i<4; i++){
        motors[i].is_starting = 1;  // 进入启动阶段
    }
    fctrl.start_tick = HAL_GetTick();  // 记录启动时刻
// 4. 设置转向模式
	fctrl.mode = MODE_TURN;
}


// 启用角度环函数
void Forward_Control_Update(void) {
    static uint8_t prev_mode = MODE_STOP;
	// 停止模式
	if(fctrl.mode == MODE_STOP){
		// 重置角度环PID积分
		fctrl.angle_pid.integral = 0;
		fctrl.angle_pid.prev_err = 0;
	// 2. 统一设置电机基准速度为0
		for(int i=0; i<4; i++){
	    	motors[i].target_speed = 0.0f;
//	    	motors[i].speed_pid.integral = 0.0f;
//	    	motors[i].is_starting = 1;
		}
		prev_mode = MODE_STOP;
//		printf("停止模式\n");
	}



	// 直线模式
	if(fctrl.mode == MODE_STRAIGHT){
//		printf("1\n");
	// 1. 准备阶段
		// 计算提前停止距离
		float stop_x = 0;
		// 如果小车不处于起步阶段
//		if(!motors[0].is_starting && !motors[1].is_starting){
//			if(fctrl.target_distance < 75.0f) {
//				if(fctrl.target_distance < 7.0f)
//					stop_x = 0;
//				else if(fctrl.target_distance < 20.f){
//					stop_x = 2.0f;
//				}
//				else
//				stop_x = 0.0489 * fctrl.target_distance + 0.333;
//			}
//			else {
//				stop_x = -0.024 * fctrl.target_distance + 4.3;
//			}
//		}
	    // 确保基准速度只在首次进入时锁定（模式切换检测）
	    static float base_speed = 0;
	    if(prev_mode != MODE_STRAIGHT) {
	        // 首次进入直线模式时，记录初始目标速度
	        base_speed = fctrl.start_base_speed;
	    }
	    prev_mode = MODE_STRAIGHT; // 更新模式状态;
	    // 动态限幅参数计算（基于 base_speed 的方向）
//	    printf("%f\n",base_speed);
	    float max_delta = MAX_SPEED_DELTA;
	    float speed_limit_upper, speed_limit_lower;
	    // 前进时
	    if(base_speed > 0) {
	        speed_limit_upper = SPEED_LIMIT_HIGH;
	        speed_limit_lower = SPEED_LIMIT_LOW;
	    }
	    // 后退时
	    else if(base_speed < 0) {
	        speed_limit_upper = -SPEED_LIMIT_LOW;
	        speed_limit_lower = -SPEED_LIMIT_HIGH;
	    } else {
	        return; // 禁止零速进入此模式
	    }
	// 2. 距离控制：计算剩余距离
        float remain_dist = fctrl.target_distance - fctrl.accumulated_dist;

	// 3. 计算Δv调整左右轮速差
        // 3.1 获取当前yaw偏差
		float current_yaw = Gyro_GetYaw();
		float yaw_err = fctrl.target_yaw - current_yaw;
//		printf("%f\n",yaw_err);

		// 3.2 角度环PID计算（输出为速度差ΔV）
		PID_Update(&fctrl.angle_pid, 0, yaw_err); // 以偏差为当前值，目标为0

		// 3.3 调整左右轮目标速度
		// 限制角度环输出的ΔV幅度（动态限幅）
		float delta_v = fctrl.angle_pid.output;
		if(delta_v > max_delta) delta_v = max_delta;
		else if(delta_v < -max_delta) delta_v = -max_delta;

	// 4.应用速度到各电机
		for (int i=0; i<4; i++) {
            if(motors[i].is_right) {
                motors[i].target_speed = base_speed - delta_v;
            } else {
                motors[i].target_speed = base_speed + delta_v;
            }
            // 接近终点时减速
			if(fctrl.enable_distance_ctrl && remain_dist < 20.0f){
//				printf("减速阶段\n");
				float reduction_ratio = (remain_dist) / 20;
				if(base_speed > 0)
					motors[i].target_speed -= (1-reduction_ratio) * 0.5 * base_speed;
				else if(base_speed < 0)
					motors[i].target_speed += (1-reduction_ratio) * 0.5 * base_speed;
			}
	        // 最终限幅（基于方向）
            motors[i].target_speed = fmaxf(speed_limit_lower, fminf(motors[i].target_speed, speed_limit_upper));
		}

	// 5. 到达终点时软停止
        if(fctrl.enable_distance_ctrl && fctrl.accumulated_dist >= fctrl.target_distance - stop_x) {
            Set_StopMode();
        }
//        printf("总目标距离=%f\n",fctrl.target_distance);
//        printf("已行驶距离=%f\n",fctrl.accumulated_dist);
//        printf("直线模式\n");
	}




	// 转向模式
	if (fctrl.mode == MODE_TURN) {
	    // 1. 获取当前偏航角和误差
	    float current_yaw = Gyro_GetYaw();
	    float yaw_err = fctrl.target_yaw - current_yaw;
		static float base_yaw = 0;
		if(prev_mode != MODE_TURN){
			base_yaw = fabs(fctrl.target_yaw - current_yaw);
		}

	    if (fabs(yaw_err) <= 2.0f) {
	    	scram();
	    	for(int i=0; i<4; i++){
	        	memset(motors[i].speed_Record, 0, sizeof(motors[i].speed_Record));
	        	motors[i].target_speed = 0.0f;
	        	motors[i].speed_pid.integral = 0;
	    	}
	        fctrl.mode = MODE_STOP; // 完成转向
	        return;
	    }

	    float direction = (yaw_err > 0) ? 1.0f : -1.0f;

	    // S形减速曲线生成
		float normalized_error = yaw_err / (base_yaw * 0.5);
		normalized_error = fminf(normalized_error, 1.0f); // 限制在[0,1]
		float x = (1.0f - 3*pow(normalized_error,2) + 2*pow(normalized_error,3));
//		printf("%f,%f",normalized_error,x);
		float base_speed = TURN_SPEED_LIMIT_HIGH * (1.0f - x);
		base_speed *= direction;
//	    printf("%f",base_speed);


	    // 3. 设置左右轮反向差速
	    for (int i = 0; i < 4; i++) {
	        if (motors[i].is_right) {
	            motors[i].target_speed = base_speed;
	        } else {
	            motors[i].target_speed = -base_speed;
	        }
		    if(motors[i].target_speed > 0 && motors[i].target_speed < TURN_SPEED_LIMIT_LOW) motors[i].target_speed = TURN_SPEED_LIMIT_LOW;
		    if(motors[i].target_speed > 0 && motors[i].target_speed > TURN_SPEED_LIMIT_HIGH) motors[i].target_speed = TURN_SPEED_LIMIT_HIGH;
		    if(motors[i].target_speed < 0 && motors[i].target_speed > -TURN_SPEED_LIMIT_LOW) motors[i].target_speed = -TURN_SPEED_LIMIT_LOW;
		    if(motors[i].target_speed < 0 && motors[i].target_speed < -TURN_SPEED_LIMIT_HIGH) motors[i].target_speed = -TURN_SPEED_LIMIT_HIGH;
	    }


//	    printf("%f %f \n",yaw_err,base_speed);
//	    printf("yaw= %f   target= %f,%f,%f,%f\n",yaw_err, motors[0].target_speed, motors[1].target_speed,motors[2].target_speed,motors[3].target_speed);
	    prev_mode = MODE_TURN;
//	    printf("%f\n",yaw_err);
//	    printf("转向模式\n");
	}

}
