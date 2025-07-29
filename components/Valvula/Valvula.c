#include "valvula.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <inttypes.h>  // <--- incluir este header

static const char *TAG = "Valvula";

void valvula_init(valvula_handle_t *handle, valvula_config_t config)
{
    handle->config = config;
    handle->actuation_count = 0;
    handle->estado_actual = false;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config.gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(config.gpio, 0);
    ESP_LOGI(TAG, "Válvula inicializada en GPIO %d", config.gpio);
}

void valvula_set(valvula_handle_t *handle, bool estado)
{
    gpio_set_level(handle->config.gpio, estado);
    if (estado && !handle->estado_actual) {
        handle->actuation_count++;
        ESP_LOGI(TAG, "Válvula ENCENDIDA (Activaciones: %" PRIu32 ")", handle->actuation_count);
    } else if (!estado && handle->estado_actual) {
        ESP_LOGI(TAG, "Válvula APAGADA");
    }
    handle->estado_actual = estado;
}

void valvula_reset_count(valvula_handle_t *handle)
{
    handle->actuation_count = 0;
    ESP_LOGI(TAG, "Contador de válvula reseteado");
}

bool valvula_get_estado(valvula_handle_t *handle)
{
    return handle->estado_actual;
}
