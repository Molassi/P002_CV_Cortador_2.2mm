/*#include "Encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

extern bool isr_service_installed;

portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

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
        .pull_up_en = GPIO_PULLUP_DISABLE,   // Pull-up activado
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
    portENTER_CRITICAL(&encoder_mux);
    handle->pulse_count = 0;
    portEXIT_CRITICAL(&encoder_mux);
}

// Devuelve la cantidad total de pulsos contados
int32_t encoder_get_pulses(encoder_handle_t *handle)
{
    int32_t count;
    portENTER_CRITICAL(&encoder_mux);
    count = handle->pulse_count;
    portEXIT_CRITICAL(&encoder_mux);
    return count;
}

// Calcula y devuelve la posición en vueltas completas (puede ser decimal)
float encoder_get_position(encoder_handle_t *handle)
{
    return (float)encoder_get_pulses(handle) / (float)handle->config.pulses_per_rev;
}

// Calcula y devuelve la distancia recorrida según la configuración
float encoder_get_distance(encoder_handle_t *handle)
{
    return encoder_get_position(handle) * handle->config.distance_per_rev;
}
*/

#include "Encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

extern bool isr_service_installed;
portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// ISR para decodificación en cuadratura
/*static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    encoder_handle_t *handle = (encoder_handle_t *)arg;

    portENTER_CRITICAL_ISR(&encoder_mux);

    // Leer estado actual de ambos pines
    uint8_t current_state = (gpio_get_level(handle->config.pin_a) << 1) | gpio_get_level(handle->config.pin_b);

    // Tabla de transiciones para cuadratura
    // (last_state << 2) | current_state representa transición
    // Suma +1 o -1 dependiendo de dirección
    int8_t delta = 0;
    switch ((handle->last_state << 2) | current_state) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            delta = +1;
            break;
        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            delta = +1;
            break;
        default:
            delta = 0; // error o rebote ignorado
            break;
    }

    handle->pulse_count += delta;
    handle->last_state = current_state;

    portEXIT_CRITICAL_ISR(&encoder_mux);
}*/

static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    encoder_handle_t *handle = (encoder_handle_t *)arg;

    portENTER_CRITICAL_ISR(&encoder_mux);

    // Leer nuevo estado de A y B
    uint8_t new_state = (gpio_get_level(handle->config.pin_a) << 1) | gpio_get_level(handle->config.pin_b);

    // Tabla de transición de cuadratura
    int8_t transition_table[16] = {
        0,  +1, -1,  0,
       -1,  0,  0, +1,
        +1, 0,  0, -1,
         0, -1, +1,  0
    };

    uint8_t transition = (handle->last_state << 2) | new_state;
    handle->pulse_count += transition_table[transition];

    handle->last_state = new_state;

    portEXIT_CRITICAL_ISR(&encoder_mux);
}

void encoder_init(encoder_handle_t *handle, encoder_config_t config)
{
    handle->config = config;
    handle->pulse_count = 0;

    // Configurar GPIO A
    gpio_config_t io_conf_A = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << handle->config.pin_a),
        .pull_up_en = GPIO_PULLUP_DISABLE, // o pull_down según tu hardware
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf_A);

    // Configurar GPIO B
    gpio_config_t io_conf_B = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << handle->config.pin_b),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf_B);

    gpio_set_direction(handle->config.pin_a, GPIO_MODE_INPUT);
    gpio_set_direction(handle->config.pin_b, GPIO_MODE_INPUT);

    if (!isr_service_installed) {
        gpio_install_isr_service(0);
        isr_service_installed = true;
        ESP_LOGI("Encoder", "ISR service instalado");
    }

    // Inicializar estado anterior leyendo los pines
    handle->last_state = (gpio_get_level(handle->config.pin_a) << 1) | gpio_get_level(handle->config.pin_b);

    // Asociar ISR a ambos pines
    gpio_isr_handler_add(handle->config.pin_a, encoder_isr_handler, (void*)handle);
    gpio_isr_handler_add(handle->config.pin_b, encoder_isr_handler, (void*)handle);
}

void encoder_reset(encoder_handle_t *handle)
{
    portENTER_CRITICAL(&encoder_mux);
    handle->pulse_count = 0;
    portEXIT_CRITICAL(&encoder_mux);
}

int32_t encoder_get_pulses(encoder_handle_t *handle)
{
    int32_t count;
    portENTER_CRITICAL(&encoder_mux);
    count = handle->pulse_count;
    portEXIT_CRITICAL(&encoder_mux);
    return count;
}

float encoder_get_position(encoder_handle_t *handle)
{
    return (float)encoder_get_pulses(handle) / (float)handle->config.pulses_per_rev;
}

float encoder_get_distance(encoder_handle_t *handle)
{
    return encoder_get_position(handle) * handle->config.distance_per_rev;
}
