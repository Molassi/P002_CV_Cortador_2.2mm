#include "Encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern bool isr_service_installed;

// Manejador de interrupción para el pin A del encoder
// Se ejecuta en ISR, por eso está en IRAM y debe ser rápida
static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    encoder_handle_t *handle = (encoder_handle_t *)arg;
    // Incrementa el contador de pulsos cada vez que detecta flanco de subida en canal A
    handle->pulse_count++;
}

// Inicializa el encoder configurando los pines GPIO y la interrupción
void encoder_init(encoder_handle_t *handle, encoder_config_t config)
{
    // Guarda la configuración en el handle
    handle->config = config;

    // Inicializa el contador de pulsos en cero
    handle->pulse_count = 0;

    // Configura el pin A como entrada con interrupción en flanco de subida
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,     // Interrupción en flanco positivo (subida)
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << handle->config.pin_a),
        .pull_up_en = GPIO_PULLUP_ENABLE,   // Pull-up activado
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    // Configura el pin B como entrada (sin interrupción, para uso futuro)
    gpio_set_direction(handle->config.pin_b, GPIO_MODE_INPUT);
    gpio_pulldown_dis(handle->config.pin_b);
    gpio_pullup_en(handle->config.pin_b);

    // Instala el servicio de ISR (si no está instalado aún)
    if (!isr_service_installed) {
        gpio_install_isr_service(0);
        isr_service_installed = true;
        ESP_LOGI("Encoder", "ISR service instalado");
    }

    // Asocia la ISR al pin A, pasando el handle como argumento
    gpio_isr_handler_add(handle->config.pin_a, encoder_isr_handler, (void*)handle);
}

// Resetea el contador de pulsos a cero
void encoder_reset(encoder_handle_t *handle)
{
    handle->pulse_count = 0;
}

// Devuelve la cantidad total de pulsos contados
int32_t encoder_get_pulses(encoder_handle_t *handle)
{
    return handle->pulse_count;
}

// Calcula y devuelve la posición en vueltas completas (puede ser decimal)
float encoder_get_position(encoder_handle_t *handle)
{
    return (float)handle->pulse_count / (float)handle->config.pulses_per_rev;
}

// Calcula y devuelve la distancia recorrida según la configuración
float encoder_get_distance(encoder_handle_t *handle)
{
    return encoder_get_position(handle) * handle->config.distance_per_rev;
}
