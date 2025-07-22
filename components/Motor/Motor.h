#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"

typedef struct {
    int pwm_gpio;
    int in1_gpio;
    int in2_gpio;
    int pwm_channel;
    int pwm_timer;
} motor_config_t;

typedef struct {
    motor_config_t config;
} motor_handle_t;

void motor_init(motor_handle_t *motor, motor_config_t config);
void motor_task(void *pvParameters);

#endif

