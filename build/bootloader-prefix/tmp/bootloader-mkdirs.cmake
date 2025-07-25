# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.2.2/components/bootloader/subproject"
  "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader"
  "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader-prefix"
  "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader-prefix/tmp"
  "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader-prefix/src"
  "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Proyectos/COLVEN/P002_Cortador_2.2mm/Software/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
