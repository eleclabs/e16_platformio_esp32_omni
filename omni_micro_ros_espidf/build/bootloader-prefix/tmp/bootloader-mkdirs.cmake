# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/eleclabs/esp/esp-idf/components/bootloader/subproject"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader-prefix"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader-prefix/tmp"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader-prefix/src/bootloader-stamp"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader-prefix/src"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
