# Omni micro-ROS ESP-IDF migration

This is the native ESP-IDF migration of the Arduino firmware in `../omni_micro_ros`.
The original PlatformIO/Arduino project is kept unchanged as a fallback.

## Target

- ESP32-S3 DevKitC-1
- ESP-IDF 5.5.2
- micro-ROS Jazzy component 24.x
- PCNT and LEDC through ESP-IDF APIs

## Build

```bash
cd ~/platformio/esp32/omni_micro_ros_espidf
source ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

Flash and monitor after connecting the board:

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

The serial device may be `/dev/ttyUSB0` instead.

## Migration status

- [x] ESP-IDF project skeleton
- [x] ESP32-S3 target configuration
- [x] Native PCNT and LEDC hardware foundation builds
- [x] micro-ROS Jazzy component resolves and builds
- [x] Native Wi-Fi network interface and UDP Agent configuration
- [x] Native micro-ROS node and wheel command/feedback topics compile
- [x] PCNT encoder counts converted to wheel velocity feedback
- [x] Native MPU6050 driver and `/imu/data_raw` publisher
- [x] Three-wheel PID calculation and 500 ms command timeout
- [ ] PID output connected to motor PWM
- [ ] Hardware validation

## Verified build result

Build verified on 2026-09-28 with ESP-IDF 5.5.2:

```text
Ninja build: SUCCESS
Binary: build/omni_micro_ros_espidf.bin
Application image: 805,504 bytes (0xC4A80)
Smallest app partition: 1,048,576 bytes
Free app partition: 23%
```

The binary now includes native Wi-Fi network initialization, micro-ROS UDP setup, PCNT encoder feedback, MPU6050 I2C/IMU publishing on `/imu/data_raw`, and a three-wheel PID calculation with a 500 ms command timeout. TB6612 `STBY` remains LOW and motor output is disabled. Configure Wi-Fi and Agent values with `idf.py menuconfig` under **micro-ROS Settings** before testing connectivity; `sdkconfig` currently contains example values only. PID output is not yet connected to motor PWM.
complete.
