#include "Motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define PWM_FREQ_HZ     1000
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT
#define PWM_MODE        LEDC_HIGH_SPEED_MODE
#define PWM_DUTY_START  60
#define PWM_DUTY_RUN    32
#define DELAY_START_MS  10
#define MOTOR_ON_MS     450
#define MOTOR_OFF_MS    1000

static void motor_start(motor_handle_t *motor)
{
    ledc_set_duty(PWM_MODE, motor->config.pwm_channel, PWM_DUTY_START);
    ledc_update_duty(PWM_MODE, motor->config.pwm_channel);
    vTaskDelay(pdMS_TO_TICKS(DELAY_START_MS));

    ledc_set_duty(PWM_MODE, motor->config.pwm_channel, PWM_DUTY_RUN);
    ledc_update_duty(PWM_MODE, motor->config.pwm_channel);
}

static void motor_stop(motor_handle_t *motor)
{
    ledc_set_duty(PWM_MODE, motor->config.pwm_channel, 0);
    ledc_update_duty(PWM_MODE, motor->config.pwm_channel);
}

void motor_init(motor_handle_t *motor, motor_config_t config)
{
    motor->config = config;

    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .timer_num        = config.pwm_timer,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num   = config.pwm_gpio,
        .speed_mode = PWM_MODE,
        .channel    = config.pwm_channel,
        .timer_sel  = config.pwm_timer,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel_conf);

    gpio_set_direction(config.in1_gpio, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.in2_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(config.in1_gpio, 1);
    gpio_set_level(config.in2_gpio, 0);
}

void motor_task(void *pvParameters)
{
    motor_handle_t *motor = (motor_handle_t *)pvParameters;

    while (1) {
        motor_start(motor);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_ON_MS));

        motor_stop(motor);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_OFF_MS));
    }
}
