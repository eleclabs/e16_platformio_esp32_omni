#pragma once

// ============================================================
// Omni 3-wheel ESP32-S3 configuration
// Keep hardware/network configuration outside main.cpp.
// ============================================================

#include <Arduino.h>

// ---------------- Network ----------------
#define OMNI_WIFI_SSID       "SK_5G"
#define OMNI_WIFI_PASSWORD   "42117622"
#define OMNI_AGENT_IP_0      192
#define OMNI_AGENT_IP_1      168
#define OMNI_AGENT_IP_2      1
#define OMNI_AGENT_IP_3      128
#define OMNI_AGENT_PORT      8888

// ---------------- Motor / wheel ----------------
constexpr float OMNI_MOTOR_RPM = 200.0f;
constexpr float OMNI_WHEEL_RADIUS_M = 0.032f;

// 500 P/R per encoder channel, quadrature x4.
// Verify this against the motor encoder datasheet and one-revolution test.
constexpr float OMNI_ENCODER_PPR = 500.0f;
constexpr float OMNI_ENCODER_COUNTS_PER_REV = OMNI_ENCODER_PPR * 4.0f;

// ---------------- PID ----------------
constexpr float OMNI_PID_KP = 0.55f;
constexpr float OMNI_PID_KI = 1.80f;
constexpr float OMNI_PID_KD = 0.00f;
constexpr float OMNI_PID_INTEGRAL_LIMIT = 8.0f;

// ---------------- Timing ----------------
constexpr uint32_t OMNI_CONTROL_PERIOD_MS = 10;       // 100 Hz
constexpr uint32_t OMNI_FEEDBACK_PERIOD_MS = 50;      // 20 Hz
constexpr uint32_t OMNI_IMU_PERIOD_MS = 10;            // 100 Hz
constexpr uint32_t OMNI_COMMAND_TIMEOUT_MS = 500;

// ---------------- PWM ----------------
constexpr uint32_t OMNI_PWM_FREQ = 20000;
constexpr uint8_t OMNI_PWM_RESOLUTION = 10;

// ---------------- TB6612 ----------------
constexpr int OMNI_PWM_PINS[3]  = {4, 5, 6};
constexpr int OMNI_DIR1_PINS[3] = {7, 15, 16};
constexpr int OMNI_DIR2_PINS[3] = {17, 18, 8};
constexpr int OMNI_STBY_PIN = 9;

// ---------------- Encoder ----------------
constexpr int OMNI_ENC_A_PINS[3] = {10, 12, 14};
constexpr int OMNI_ENC_B_PINS[3] = {11, 13, 21};

// Change only after physical direction testing.
constexpr int OMNI_MOTOR_SIGN[3] = {+1, +1, +1};
constexpr int OMNI_ENCODER_SIGN[3] = {+1, +1, +1};

// ---------------- MPU6050 ----------------
constexpr int OMNI_I2C_SDA_PIN = 1;
constexpr int OMNI_I2C_SCL_PIN = 2;
constexpr uint8_t OMNI_MPU6050_ADDRESS = 0x68;
constexpr uint16_t OMNI_IMU_CALIBRATION_SAMPLES = 500;
