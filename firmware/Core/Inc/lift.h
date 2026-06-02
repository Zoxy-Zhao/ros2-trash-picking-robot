#ifndef INC_LIFT_H_
#define INC_LIFT_H_

#include "main.h"

#define DIR_PIN    GPIO_PIN_10    // PE10（方向信号）
#define EN_PIN     GPIO_PIN_11    // PE11（使能信号）
#define GPIO_PORT  GPIOE



void lift_Init();
void get_in_right();
void get_out_right();
void get_in_left();
void get_out_left();

#endif /* INC_LIFT_H_ */
