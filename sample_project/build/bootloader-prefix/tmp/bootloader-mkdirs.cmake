# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.2.2/components/bootloader/subproject"
  "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader"
  "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader-prefix"
  "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader-prefix/tmp"
  "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader-prefix/src"
  "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Documentos/UFMG/PFC/ESP32/Projetos/PFC/sample_project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
