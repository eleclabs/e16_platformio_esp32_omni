# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "bootloader/bootloader.bin"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.map"
  "config/sdkconfig.cmake"
  "config/sdkconfig.h"
  "esp-idf/esptool_py/flasher_args.json.in"
  "esp-idf/mbedtls/x509_crt_bundle"
  "flash_app_args"
  "flash_bootloader_args"
  "flash_project_args"
  "flasher_args.json"
  "ldgen_libraries"
  "ldgen_libraries.in"
  "omni_micro_ros_espidf.bin"
  "omni_micro_ros_espidf.map"
  "project_elf_src_esp32s3.c"
  "x509_crt_bundle.S"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/managed_components/micro-ros__micro_ros_espidf_component/esp32_toolchain.cmake"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/managed_components/micro-ros__micro_ros_espidf_component/include"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/managed_components/micro-ros__micro_ros_espidf_component/micro_ros_dev"
  "/home/eleclabs/platformio/esp32/omni_micro_ros_espidf/managed_components/micro-ros__micro_ros_espidf_component/micro_ros_src"
  )
endif()
