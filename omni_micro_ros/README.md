# Omni 3-Wheel ESP32-S3 micro-ROS Base Controller

Firmware สำหรับ Omni 3-Wheel / Kiwi Drive:

- MCU: ESP32-S3
- Motor: DC 12 V, 200 RPM
- Encoder: Quadrature AB, 500 P/R (assumed per channel before x4 decoding)
- Driver: 3 × TB6612
- Control: 3 independent wheel PID velocity loops
- Encoder: ESP-IDF PCNT hardware peripheral, x4 quadrature decoding
- ROS 2 transport: micro-ROS over Wi-Fi UDP
- Command: `/wheel_velocity_cmd`
- Feedback: `/wheel_velocity_feedback`
- Safety watchdog: 500 ms command timeout
- IMU: MPU6050 over I2C
- IMU topic: `/imu/data_raw` (`sensor_msgs/msg/Imu`)
- IMU rate: 100 Hz
- Gyro stationary bias calibration at boot

## 1. Architecture

```text
ROS 2 Jazzy
    │
    │ /wheel_velocity_cmd [rad/s]
    ▼
micro-ROS Agent :8888/UDP
    │
    ▼
ESP32-S3
    │
    ├── PCNT Encoder W1 ──┐
    ├── PCNT Encoder W2 ──┼── measured wheel velocity
    └── PCNT Encoder W3 ──┘
              │
              ▼
        PID velocity ×3
              │
              ▼
       TB6612 ×3 → Motors
              │
              ▼
/wheel_velocity_feedback [rad/s]
```

## 2. Wheel Order

The firmware uses the same order as the ROS 2 controller:

```text
W1 = Front Left
W2 = Front Right
W3 = Rear Center
```

Do not change this order between URDF, controller, ESP32 and odometry.

## 3. Encoder Resolution

The firmware currently assumes:

```text
500 P/R/channel × quadrature x4 = 2000 counts/revolution
```

If the motor datasheet defines **500 CPR as the final quadrature count**, change:

```cpp
QUADRATURE_COUNTS_PER_REV = 500.0f;
```

The correct value should be verified by commanding one wheel to rotate exactly one mechanical revolution and checking the PCNT count.

If the encoder is mounted before a gearbox, include the gearbox ratio in the effective counts/revolution.

## 4. Motor Limit

200 RPM:

```text
200 × 2π / 60 = 20.944 rad/s
```

The firmware limits commanded wheel velocity to approximately:

```text
±20.94 rad/s
```

This is the nominal no-load motor speed. Actual loaded speed is lower.

## 5. TB6612 Wiring

Initial GPIO map in `src/main.cpp`:

| Wheel | PWM | DIR1 | DIR2 | Encoder A | Encoder B |
|---|---:|---:|---:|---:|---:|
| W1 | GPIO4 | GPIO7 | GPIO17 | GPIO10 | GPIO11 |
| W2 | GPIO5 | GPIO15 | GPIO18 | GPIO12 | GPIO13 |
| W3 | GPIO6 | GPIO16 | GPIO8 | GPIO14 | GPIO21 |

Common:

```text
TB6612 STBY → GPIO9
ESP32 GND   → Driver GND
Motor VM    → 12 V motor supply
ESP32 VCC   → 3.3 V logic
```

**Verify the exact ESP32-S3 board pinout and TB6612 module wiring before applying motor power.**

TB6612 logic and motor supply grounds must be common.

## 6. Direction Polarity

The following arrays allow each wheel to be reversed without changing ROS 2 kinematics:

```cpp
MOTOR_SIGN[3]   = {+1, +1, +1};
ENCODER_SIGN[3] = {+1, +1, +1};
```

Bench-test each wheel independently and change only the affected sign when necessary.

## 7. PID

Initial conservative values:

```cpp
Kp = 0.55
Ki = 1.80
Kd = 0.00
```

PID runs at 100 Hz.

Do not tune PID with the robot on the floor initially. Lift the wheels, command a small velocity, observe feedback, then tune one wheel at a time.

The controller uses:

```text
output = target + Kp*error + Ki*integral + Kd*derivative
```

with integral limiting.

## 8. Safety Watchdog

If `/wheel_velocity_cmd` is not received for more than:

```text
500 ms
```

the firmware sets all targets to zero, resets the PID integrators and disables motor PWM.

TB6612 `STBY` is driven LOW only for fatal initialization errors; normal command timeout stops PWM and clears the motor direction outputs.

## 9. MPU6050 Wiring

Default I2C mapping in `src/main.cpp`:

| MPU6050 | ESP32-S3 |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SDA | GPIO1 |
| SCL | GPIO2 |
| AD0 | GND → address `0x68` |

If AD0 is HIGH, change the address to `0x69`.

The firmware configures:

```text
Accelerometer = ±8 g
Gyroscope     = ±500 deg/s
Digital LPF   = 21 Hz
Sample        = 100 Hz
```

At boot the robot must remain stationary for about 1 second while gyro bias is measured. Do not move the robot during this calibration.

The MPU6050 has an accelerometer and gyroscope but **no magnetometer**, so it cannot provide absolute yaw by itself. The firmware therefore publishes raw IMU data and marks orientation as unknown. Do not use the identity quaternion as a measured orientation in EKF.

ROS 2 topic:

```bash
ros2 topic echo /imu/data_raw
```

Expected message type:

```text
sensor_msgs/msg/Imu
```

## 10. PlatformIO Project Structure

The ESP32 firmware is a standalone PlatformIO project, intentionally outside the ROS 2 `colcon` workspace:

```text
~/platformio/esp32/omni_micro_ros/
├── platformio.ini
├── include/
│   └── omni_config.h
├── src/
│   └── main.cpp
├── README.md
└── .gitignore
```

Hardware, PID, timing, encoder and I2C settings are centralized in `include/omni_config.h`.

Build from WSL:

```bash
cd ~/platformio/esp32/omni_micro_ros
pio run
```

Upload:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor -b 115200
```

Before flashing, replace the Wi-Fi and Agent placeholders in `include/omni_config.h`:

```cpp
OMNI_WIFI_SSID
OMNI_WIFI_PASSWORD
OMNI_AGENT_IP_0 ... OMNI_AGENT_IP_3
OMNI_AGENT_PORT
```

Do not commit real Wi-Fi credentials. A future production setup should move them to an ignored local `secrets.h`.

Open this directory in VS Code / PlatformIO:

```text
~/platformio/esp32/omni_micro_ros
```

Build:

```bash
pio run
```

Upload:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor -b 115200
```

Before building, edit `src/main.cpp`:

```cpp
WIFI_SSID
WIFI_PASSWORD
AGENT_IP
AGENT_PORT
```

## 11. micro-ROS Agent on E16

On the ROS 2 host/container:

```bash
docker exec -it ros2_jazzy bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

Start UDP agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

The ESP32 must be on the same LAN and `AGENT_IP` must point to the machine/container endpoint reachable from the ESP32.

If the agent runs inside Docker, UDP port 8888 must be exposed/reachable from the LAN.

## 12. Verify ROS 2

After ESP32 connects:

```bash
ros2 node list
ros2 topic list | grep wheel
```

Expected feedback topic:

```text
/wheel_velocity_feedback
```

Inspect:

```bash
ros2 topic echo /wheel_velocity_feedback
```

## 13. Test One Wheel

With wheels lifted from the floor:

```bash
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray \\
  "{data: [2.0, 0.0, 0.0]}"
```

Only W1 should move.

Then stop:

```bash
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray \\
  "{data: [0.0, 0.0, 0.0]}"
```

Repeat for W2 and W3.

## 14. Encoder Validation

For each wheel:

1. Keep motor disabled.
2. Rotate the wheel manually one mechanical revolution.
3. Read PCNT count through the firmware debug output if enabled.
4. Compare measured counts with `QUADRATURE_COUNTS_PER_REV`.
5. Confirm positive mechanical rotation produces positive feedback.

Do not proceed to floor testing until encoder sign and resolution are correct.

## 15. ROS 2 Controller Integration

Data path:

```text
/cmd_vel
   ↓
omni_kinematics_node
   ↓
/wheel_velocity_cmd
   ↓
micro-ROS
   ↓
ESP32 PID + TB6612
   ↓
Encoder
   ↓
/wheel_velocity_feedback
   ↓
omni_odometry_node
   ↓
/odom
```

The ESP32 does not run SLAM or Nav2.

## 16. Bring-up Sequence

Use this order:

```text
1. Verify GPIO wiring without motor power
2. Verify encoder counts manually
3. Start micro-ROS Agent
4. Connect ESP32
5. Verify /wheel_velocity_feedback
6. Test W1 at low speed
7. Test W2 at low speed
8. Test W3 at low speed
9. Verify motor/encoder signs
10. Tune PID
11. Test /wheel_velocity_cmd from ROS 2
12. Connect omni_odometry_node
13. Test /odom
14. Only then test the robot on the floor
15. Continue to EKF → SLAM → Nav2
```

## 17. Important Electrical Check

TB6612 is suitable only when the motor's actual operating/stall current is within the driver's limits. A 12 V 200 RPM motor can have a stall current much higher than its running current. Measure/check the motor datasheet before full-load testing.

Never power the motor from the ESP32 5 V/3.3 V rail.

## 18. ROS 2 Real-Hardware Localization

For the real ESP32-S3 robot, `/wheel_velocity_feedback` must have **one publisher only**: the ESP32 micro-ROS node. The ROS 2 `omni_hardware_interface` simulation node must not be started because it would create a second publisher on the same feedback topic.

The controller launch now defaults to real hardware mode:

```bash
ros2 launch omni_robot_controller controller.launch.py
```

For simulation/open-loop testing only:

```bash
ros2 launch omni_robot_controller controller.launch.py use_simulation:=true
```

The real localization stack is:

```bash
ros2 launch omni_robot_localization localization.launch.py
```

This starts the wheel kinematics/odometry nodes and `robot_localization` EKF. The EKF is the **single owner of `odom -> base_link` TF**. `omni_odometry_node` publishes `/odom` but does not publish TF by default.

Verify the TF chain:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

Expected:

```text
odom → base_link → imu_link
```

Verify topics:

```bash
ros2 topic info /wheel_velocity_feedback -v
ros2 topic info /odom -v
ros2 topic info /imu/data_raw -v
ros2 topic info /odometry/filtered -v
```

For real hardware, `/wheel_velocity_feedback` should show the ESP32 micro-ROS node as its publisher. There should not be an `omni_hardware_interface` publisher.

## 19. Code Review: Required Checks and Recommended Improvements

The source code is a firmware scaffold with the main control path implemented, but it has not been verified against the physical robot in this workspace. Complete these checks before floor testing:

### Required before flashing

1. Replace `WIFI_SSID`, `WIFI_PASSWORD`, and `AGENT_IP` in `src/main.cpp`. Do not commit real credentials to Git.
2. Verify the ESP32-S3 board revision and every GPIO against the actual wiring. Some ESP32-S3 boards reserve or expose GPIOs differently.
3. Confirm encoder resolution, gearbox ratio, and the sign of each motor/encoder. The current values are assumptions.
4. Confirm the micro-ROS Agent UDP port is reachable from the ESP32, including Docker port/network settings.
5. Install PlatformIO in the WSL environment and run `pio run`; the current environment reported `pio: command not found`, so compilation is not yet verified.

### Recommended code improvements

- Add Wi-Fi connection and micro-ROS Agent reconnect handling instead of stopping permanently after initialization failure.
- Validate command values with `isfinite()` before using them, and reject NaN/Inf values from `/wheel_velocity_cmd`.
- Keep TB6612 `STBY` LOW until all required initialization and ROS communication checks pass; this reduces startup motion risk.
- Add a `dt` lower-bound check before dividing in velocity/PID calculations, and improve PID anti-windup if the output saturates frequently.
- Publish or log encoder counts and connection state during bring-up to simplify calibration and fault diagnosis.
- Add a startup configuration section or generated local config so network credentials are separated from version-controlled source.
- Add cleanup/error logging for failed ROS allocations and initialization calls; `RCCHECK` currently enters an infinite fatal-stop loop.

These are the highest-priority items found by static review. A successful build and hardware test are still required before treating the firmware as production-ready.

## 20. Quick Usage Checklist

```bash
cd ~/platformio/esp32/omni_micro_ros
source /opt/ros/jazzy/setup.bash
pio run
pio run -t upload
pio device monitor -b 115200
```

Before uploading, keep the robot wheels lifted and verify the serial output. Start the Agent first:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Then verify the connection and topics:

```bash
ros2 node list
ros2 topic list | grep -E 'wheel|imu'
ros2 topic echo /wheel_velocity_feedback
ros2 topic echo /imu/data_raw
```

Send only low-speed one-wheel commands during the first test:

```bash
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray \\
  "{data: [1.0, 0.0, 0.0]}"
```

Stop immediately with:

```bash
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray \\
  "{data: [0.0, 0.0, 0.0]}"
```

## 21. PlatformIO Migration Status

```text
ESP32 firmware standalone in PlatformIO    → complete
Centralized hardware configuration          → complete
MPU6050 + micro-ROS + PCNT + PID            → integrated
NaN/Inf command protection                  → added
PID dt safety check                         → added
Physical motor/encoder calibration          → next
```

## 22. Current Phase

```text
Phase 3A  Controller validation      → in progress
Phase 3.1 ESP32 + micro-ROS          → firmware scaffold implemented
Phase 3.2 Real encoder validation    → next
Phase 3.3 MPU6050 IMU validation      → implemented
Phase 3.4 EKF-ready IMU + odometry    → next
Phase 4   EKF                         → pending
Phase 5   SLAM Toolbox               → pending
Phase 7   Nav2                       → pending
```
