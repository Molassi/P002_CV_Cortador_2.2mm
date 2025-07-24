/* --- FACIL - PRIMER PROYECTO CON ESP-IDF ---
#include <stdio.h>
#include "driver/gpio.h" //Driver para manejar gpios.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    //  Tnitializing GPIO
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);


    while(true){
        gpio_set_level(GPIO_NUM_5, 0);
        vTaskDelay(pdMS_TO_TICKS(5000));
        gpio_set_level(GPIO_NUM_5, 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

}
*/


/* --- FUNCIONA --- 
       SOLO QUE AHORA VOY A HACER UNO DE FORMA MODULAR PARA PODER RECICLAR CLIDGOS.
       -> PARA ESTO SE NECESITA AGREGAR UNA CARPETA Y DEMAS YERBAS.
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define PWM_GPIO        GPIO_NUM_13             // Pin de salida PWM
#define PWM_FREQ_HZ     1000                    // Frecuencia en Hz
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT        // Resolución: 8 bits (0-255)
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_HIGH_SPEED_MODE
#define PWM_CHANNEL     LEDC_CHANNEL_0
#define PWM_DUTY_START  60 //128                // Duty Cycle en inicio (10ms). (50% si es 8-bit, 128/255)
#define PWM_DUTY_RUN    32                      // Duty cycle nominal.
#define DELAY_START_MS  10                      // Tiempo para romper inercia.
#define MOTOR_ON_MS     450                    // Tiempo que el motor permanece prendido.
#define MOTOR_OFF_MS    1000                    // Tiempo que el motor permanece apagado. 

// Pines de direccion.
#define IN1_GPIO GPIO_NUM_25
#define IN2_GPIO GPIO_NUM_26

void motor_start()
{
    // Empujón inicial
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, PWM_DUTY_START);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(DELAY_START_MS));

    // Mantener velocidad baja
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, PWM_DUTY_RUN);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

void motor_stop()
{
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

void app_main(void)
{
    // Configurar timer PWM
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // Configurar canal PWM
    ledc_channel_config_t channel_conf = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel_conf);

    // Configurar pines de dirección
    gpio_set_direction(IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2_GPIO, GPIO_MODE_OUTPUT);

    // Establecer dirección (ejemplo: IN1=1, IN2=0)
    gpio_set_level(IN1_GPIO, 1);
    gpio_set_level(IN2_GPIO, 0);

    while (true)
    {
        // Encender motor con empujón y velocidad baja
        motor_start();
        vTaskDelay(pdMS_TO_TICKS(MOTOR_ON_MS));

        // Apagar motor
        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(MOTOR_OFF_MS));
    }
}
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#include "Motor.h"
#include "Encoder.h"

motor_handle_t motor1;
encoder_handle_t encoder1;
//motor_handle_t motor2;

void app_main(void)
{
    motor_config_t motor1_config = {
        .pwm_gpio = 13,
        .in1_gpio = 25,
        .in2_gpio = 26,
        .pwm_channel = LEDC_CHANNEL_0,
        .pwm_timer = LEDC_TIMER_0
    };

    encoder_config_t encoder1_config = {
        .pin_a = 34,
        .pin_b = 35,
        .pulses_per_rev = 600,
        .distance_per_rev = 10.050
    };

    /*motor_config_t config2 = {
        .pwm_gpio = 14,
        .in1_gpio = 32,
        .in2_gpio = 33,
        .pwm_channel = LEDC_CHANNEL_1,
        .pwm_timer = LEDC_TIMER_1
    };*/

    motor_init(&motor1, motor1_config);
    encoder_init(&encoder1, encoder1_config);
    //_init(&motor2, config2);

    xTaskCreate(motor_task, "motor1_task", 2048, &motor1, 5, NULL);
    //xTaskCreate(motor_task, "motor2_task", 2048, &motor2, 5, NULL);
}
