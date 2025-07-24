#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#include "Motor.h"
#include "Encoder.h"

extern motor_handle_t motor1;
extern encoder_handle_t encoder1;

void task_control(void *pvParameters)
{
    motor_config_t motor1_config = {
        .pwm_gpio = 13,
        .in1_gpio = 25,
        .in2_gpio = 26,
        .pwm_channel = LEDC_CHANNEL_0,
        .pwm_timer = LEDC_TIMER_0,
        .pwm_duty_start = 60,
        .pwm_duty_run = 32
    };

    encoder_config_t encoder1_config = {
        .pin_a = 34,
        .pin_b = 35,
        .pulses_per_rev = 600,
        .distance_per_rev = 10.050
    };

    motor_init(&motor1, motor1_config);
    encoder_init(&encoder1, encoder1_config);

    while (1) {
        motor_start(&motor1);
        vTaskDelay(pdMS_TO_TICKS(450)); //elimine la definicion de tiempo

        motor_stop(&motor1);
        vTaskDelay(pdMS_TO_TICKS(1000)); //elimine la definicion de tiempo
    }
}