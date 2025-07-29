/* ARCHIVO PRINCIPAL DEL PROYECTO
    Función: inicializar cosas principales del sistema.
    i)  Inicializar los componentes.
    ii) Crear tareas que definen el comportamiento del sistema.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#include "Motor.h"
#include "Encoder.h"
#include "Valvula.h"
#include "task_control.h"

motor_handle_t motor1;
encoder_handle_t encoder1;
valvula_handle_t valvula1;

bool isr_service_installed = false;

void app_main(void)
{
    xTaskCreate(task_control, "Tarea principal", 2048, NULL, 5, NULL);  //Puntero de tarea, nombre descriptivo, tamaño pila RAM asignada, parametro que se pasa, prioridad, puntero donde se guarda el dandle de la tarea.
}
