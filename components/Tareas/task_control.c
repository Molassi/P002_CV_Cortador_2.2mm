#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/portmacro.h"

#include "Motor.h"
#include "Encoder.h"
#include "Valvula.h"

extern motor_handle_t motor1;
extern encoder_handle_t encoder1;
extern valvula_handle_t valvula1;
extern portMUX_TYPE encoder_mux;

static const char *TAG = "Tareas";

void task_control(void *pvParameters)
{
    motor_config_t motor1_config = {
        .pwm_gpio = 13,
        .in1_gpio = 25,
        .in2_gpio = 26,
        .pwm_channel = LEDC_CHANNEL_0,
        .pwm_timer = LEDC_TIMER_0,
        .pwm_duty_start = 60,
        .pwm_duty_run = 45
    };

    encoder_config_t encoder1_config = {
        .pin_a = 14,
        .pin_b = 27,
        .pulses_per_rev = 2400,
        .distance_per_rev = 10.050
    };

    valvula_config_t valvula1_config = {
        .gpio = 25
    };

    motor_init(&motor1, motor1_config);
    encoder_init(&encoder1, encoder1_config);
    valvula_init(&valvula1, valvula1_config);

    int32_t pulsos_actuales;
    int32_t cant_cortes = 500;

    valvula_set(&valvula1, false);  //apago valvula. (levantar)
    valvula_reset_count(&valvula1); //reset de contador

    while (valvula1.actuation_count < cant_cortes) {
        
        //En teoría necesito 2396 pulsos - el motor da una vuelta.
        motor_start(&motor1);
    
        portENTER_CRITICAL(&encoder_mux);
        pulsos_actuales = encoder1.pulse_count;
        portEXIT_CRITICAL(&encoder_mux);
        
        if (pulsos_actuales >= 67) {
            motor_stop(&motor1);
            ESP_LOGI(TAG, "Pulsos contados: %" PRId32, pulsos_actuales);
            encoder_reset(&encoder1);
            vTaskDelay(pdMS_TO_TICKS(300));

            valvula_set(&valvula1, true); //acitvo valvula. (subir)
            vTaskDelay(pdMS_TO_TICKS(600));
            valvula_set(&valvula1, false); //apago valvula. (levantar)
            vTaskDelay(pdMS_TO_TICKS(600));

        }
        
    }

    while (valvula1.actuation_count >= cant_cortes) {} // Tienen que reiniciar.
}