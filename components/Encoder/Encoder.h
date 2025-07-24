#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
#include <stdint.h>

// Estructura para configuración del encoder
typedef struct {
    gpio_num_t pin_a;          // Pin del canal A
    gpio_num_t pin_b;          // Pin del canal B (para futuro uso)
    int pulses_per_rev;        // Pulsos por revolución del encoder
    float distance_per_rev;    // Distancia física por vuelta (mm, cm, etc)
} encoder_config_t;

// Estructura para manejar el estado del encoder
typedef struct {
    encoder_config_t config;   // Parámetros de configuración
    volatile int32_t pulse_count;  // Contador de pulsos (volatile porque se modifica en ISR)
} encoder_handle_t;

// Inicializa el encoder con la configuración dada
void encoder_init(encoder_handle_t *handle, encoder_config_t config);

// Resetea el contador de pulsos a cero
void encoder_reset(encoder_handle_t *handle);

// Retorna la cantidad actual de pulsos contados
int32_t encoder_get_pulses(encoder_handle_t *handle);

// Retorna la posición en vueltas (puede ser un número fraccional)
float encoder_get_position(encoder_handle_t *handle);

// Retorna la distancia recorrida según la configuración
float encoder_get_distance(encoder_handle_t *handle);

#endif // ENCODER_H
