idf_component_register(SRCS "task_control.c"                       
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES Encoder Motor Tareas freertos driver Valvula)