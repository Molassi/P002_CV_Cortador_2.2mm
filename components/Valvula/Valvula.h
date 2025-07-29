#ifndef VALVULA_H
#define VALVULA_H

#include "driver/gpio.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    gpio_num_t gpio;             // GPIO de control
} valvula_config_t;

typedef struct {
    valvula_config_t config;
    uint32_t actuation_count;    // Veces que se activó la válvula
    bool estado_actual;          // true: abierta, false: cerrada
} valvula_handle_t;

void valvula_init(valvula_handle_t *handle, valvula_config_t config);
void valvula_set(valvula_handle_t *handle, bool estado);
void valvula_toggle(valvula_handle_t *handle);
void valvula_reset_count(valvula_handle_t *handle);
bool valvula_get_estado(valvula_handle_t *handle);

#endif