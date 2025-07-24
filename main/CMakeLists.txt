# "CMakeLists.txt" le dice al sitema de compilacion (CMake) como compilar el componente o aplicación.
# Componente: es una carpeta que contiene código reutilizable, por ejemplo: Motor.
# Cada componente puede declarar: i) que archivos fuente compila(SRCS). ii)Que directorios de cabecera expone(INCLUDE_DIRS)
# iii) Dependencias de otros compoenentes (REQUIRES, PRIV_REQUIRES)
idf_component_register(SRCS "P002_CV.c"                         # archivo principal.
                       INCLUDE_DIRS "."
                       REQUIRES driver Encoder Motor Tareas freertos)
