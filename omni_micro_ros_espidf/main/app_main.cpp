#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uros_network_interfaces.h"
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/float64_multi_array.h"
#include "sensor_msgs/msg/imu.h"

namespace {

constexpr char kTag[] = "omni_espidf";
constexpr gpio_num_t kPwmPins[3] = {
    GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6};
constexpr gpio_num_t kDir1Pins[3] = {
    GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16};
constexpr gpio_num_t kDir2Pins[3] = {
    GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_8};
constexpr gpio_num_t kEncoderAPins[3] = {
    GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_14};
constexpr gpio_num_t kEncoderBPins[3] = {
    GPIO_NUM_11, GPIO_NUM_13, GPIO_NUM_21};
constexpr gpio_num_t kStandbyPin = GPIO_NUM_9;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr ledc_timer_bit_t kPwmResolution = LEDC_TIMER_10_BIT;
constexpr float kEncoderCountsPerRevolution = 2000.0f;
constexpr float kControlPeriodSeconds = 0.01f;
constexpr float kMotorRpm = 200.0f;
constexpr float kMaxWheelRadPerSec = kMotorRpm * 2.0f * 3.14159265358979323846f / 60.0f;
constexpr float kPidKp = 0.55f;
constexpr float kPidKi = 1.80f;
constexpr float kPidKd = 0.00f;
constexpr float kPidIntegralLimit = 8.0f;
constexpr uint32_t kCommandTimeoutMs = 500;
constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr gpio_num_t kI2cSdaPin = GPIO_NUM_1;
constexpr gpio_num_t kI2cSclPin = GPIO_NUM_2;
constexpr uint8_t kMpuAddress = 0x68;
constexpr int kMpuCalibrationSamples = 100;
constexpr float kGravity = 9.80665f;

pcnt_unit_handle_t gPcntUnits[3] = {};
pcnt_channel_handle_t gPcntAChannels[3] = {};
pcnt_channel_handle_t gPcntBChannels[3] = {};

rcl_node_t gNode = rcl_get_zero_initialized_node();
rcl_publisher_t gFeedbackPublisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t gImuPublisher = rcl_get_zero_initialized_publisher();
rcl_subscription_t gCommandSubscription = rcl_get_zero_initialized_subscription();
rclc_executor_t gExecutor = rclc_executor_get_zero_initialized_executor();
std_msgs__msg__Float64MultiArray gCommandMessage;
std_msgs__msg__Float64MultiArray gFeedbackMessage;
sensor_msgs__msg__Imu gImuMessage;
double gCommandValues[3] = {0.0, 0.0, 0.0};
double gFeedbackValues[3] = {0.0, 0.0, 0.0};
float gTargetRadPerSec[3] = {0.0f, 0.0f, 0.0f};
float gPidIntegral[3] = {0.0f, 0.0f, 0.0f};
float gPidPreviousError[3] = {0.0f, 0.0f, 0.0f};
float gMotorCommandRadPerSec[3] = {0.0f, 0.0f, 0.0f};
float gGyroBias[3] = {0.0f, 0.0f, 0.0f};
int64_t gLastCommandUs = 0;

void CheckRcl(rcl_ret_t result, const char *operation) {
  if (result != RCL_RET_OK) {
    ESP_LOGE(kTag, "%s failed: %d", operation, static_cast<int>(result));
    vTaskDelete(nullptr);
  }
}

void CommandCallback(const void *message) {
  const auto *command = static_cast<const std_msgs__msg__Float64MultiArray *>(message);
  if (command->data.size < 3 || command->data.data == nullptr) return;
  for (size_t i = 0; i < 3; ++i) {
    if (!isfinite(command->data.data[i])) return;
    gCommandValues[i] = command->data.data[i];
    gTargetRadPerSec[i] = constrain(static_cast<float>(command->data.data[i]),
                                    -kMaxWheelRadPerSec, kMaxWheelRadPerSec);
  }
  gLastCommandUs = esp_timer_get_time();
  ESP_LOGI(kTag, "Received wheel command [%.2f %.2f %.2f] (motors disabled)",
           gCommandValues[0], gCommandValues[1], gCommandValues[2]);
}

void CheckError(esp_err_t error, const char *operation) {
  if (error != ESP_OK) {
    ESP_LOGE(kTag, "%s failed: %s", operation, esp_err_to_name(error));
    abort();
  }
}

void InitPins() {
  gpio_config_t direction_config = {};
  direction_config.pin_bit_mask = 0;
  for (int i = 0; i < 3; ++i) {
    direction_config.pin_bit_mask |= 1ULL << kDir1Pins[i];
    direction_config.pin_bit_mask |= 1ULL << kDir2Pins[i];
  }
  direction_config.mode = GPIO_MODE_OUTPUT;
  CheckError(gpio_config(&direction_config), "gpio_config(direction)");

  gpio_config_t standby_config = {};
  standby_config.pin_bit_mask = 1ULL << kStandbyPin;
  standby_config.mode = GPIO_MODE_OUTPUT;
  CheckError(gpio_config(&standby_config), "gpio_config(standby)");
  CheckError(gpio_set_level(kStandbyPin, 0), "gpio_set_level(standby)");
}

void InitPwm() {
  ledc_timer_config_t timer_config = {};
  timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_config.timer_num = LEDC_TIMER_0;
  timer_config.duty_resolution = kPwmResolution;
  timer_config.freq_hz = kPwmFrequencyHz;
  timer_config.clk_cfg = LEDC_AUTO_CLK;
  CheckError(ledc_timer_config(&timer_config), "ledc_timer_config");

  for (int i = 0; i < 3; ++i) {
    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = kPwmPins[i];
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = static_cast<ledc_channel_t>(LEDC_CHANNEL_0 + i);
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = LEDC_TIMER_0;
    channel_config.duty = 0;
    channel_config.hpoint = 0;
    CheckError(ledc_channel_config(&channel_config), "ledc_channel_config");
  }
}

void InitEncoders() {
  for (int i = 0; i < 3; ++i) {
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32768;
    unit_config.high_limit = 32767;
    CheckError(pcnt_new_unit(&unit_config, &gPcntUnits[i]), "pcnt_new_unit");

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    CheckError(pcnt_unit_set_glitch_filter(
        gPcntUnits[i], &filter_config), "pcnt_unit_set_glitch_filter");

    pcnt_chan_config_t a_config = {};
    a_config.edge_gpio_num = kEncoderAPins[i];
    a_config.level_gpio_num = kEncoderBPins[i];
    CheckError(pcnt_new_channel(
        gPcntUnits[i], &a_config, &gPcntAChannels[i]), "pcnt_new_channel(A)");
    CheckError(pcnt_channel_set_edge_action(
        gPcntAChannels[i], PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE), "pcnt edge action(A)");
    CheckError(pcnt_channel_set_level_action(
        gPcntAChannels[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE), "pcnt level action(A)");

    pcnt_chan_config_t b_config = {};
    b_config.edge_gpio_num = kEncoderBPins[i];
    b_config.level_gpio_num = kEncoderAPins[i];
    CheckError(pcnt_new_channel(
        gPcntUnits[i], &b_config, &gPcntBChannels[i]), "pcnt_new_channel(B)");
    CheckError(pcnt_channel_set_edge_action(
        gPcntBChannels[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE), "pcnt edge action(B)");
    CheckError(pcnt_channel_set_level_action(
        gPcntBChannels[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE), "pcnt level action(B)");

    CheckError(pcnt_unit_enable(gPcntUnits[i]), "pcnt_unit_enable");
    CheckError(pcnt_unit_clear_count(gPcntUnits[i]), "pcnt_unit_clear_count");
    CheckError(pcnt_unit_start(gPcntUnits[i]), "pcnt_unit_start");
  }
}

}  // namespace

void I2cWriteByte(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  CheckError(i2c_master_write_to_device(kI2cPort, kMpuAddress, data,
                                        sizeof(data), pdMS_TO_TICKS(100)),
             "i2c_master_write_to_device");
}

void I2cRead(uint8_t reg, uint8_t *data, size_t length) {
  CheckError(i2c_master_write_read_device(kI2cPort, kMpuAddress, &reg, 1,
                                          data, length, pdMS_TO_TICKS(100)),
             "i2c_master_write_read_device");
}

int16_t ReadInt16(const uint8_t *data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

void InitMpu6050() {
  i2c_config_t config = {};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = kI2cSdaPin;
  config.scl_io_num = kI2cSclPin;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = 400000;
  CheckError(i2c_param_config(kI2cPort, &config), "i2c_param_config");
  CheckError(i2c_driver_install(kI2cPort, config.mode, 0, 0, 0),
             "i2c_driver_install");

  uint8_t whoAmI = 0;
  I2cRead(0x75, &whoAmI, 1);
  if (whoAmI != 0x68) {
    ESP_LOGE(kTag, "MPU6050 WHO_AM_I mismatch: 0x%02x", whoAmI);
    abort();
  }
  I2cWriteByte(0x6B, 0x00);
  I2cWriteByte(0x1C, 0x10);  // accelerometer +/- 8 g
  I2cWriteByte(0x1B, 0x08);  // gyroscope +/- 500 deg/s
  I2cWriteByte(0x1A, 0x04);  // digital low-pass filter

  double bias[3] = {0.0, 0.0, 0.0};
  for (int sample = 0; sample < kMpuCalibrationSamples; ++sample) {
    uint8_t raw[14] = {};
    I2cRead(0x3B, raw, sizeof(raw));
    bias[0] += static_cast<double>(ReadInt16(raw + 8)) * (500.0 / 32768.0) *
               3.14159265358979323846 / 180.0;
    bias[1] += static_cast<double>(ReadInt16(raw + 10)) * (500.0 / 32768.0) *
               3.14159265358979323846 / 180.0;
    bias[2] += static_cast<double>(ReadInt16(raw + 12)) * (500.0 / 32768.0) *
               3.14159265358979323846 / 180.0;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  for (int axis = 0; axis < 3; ++axis) {
    gGyroBias[axis] = static_cast<float>(bias[axis] / kMpuCalibrationSamples);
  }
  ESP_LOGI(kTag, "MPU6050 ready; gyro bias [%.5f %.5f %.5f] rad/s",
           gGyroBias[0], gGyroBias[1], gGyroBias[2]);
}

void UpdateImuMessage() {
  uint8_t raw[14] = {};
  I2cRead(0x3B, raw, sizeof(raw));
  const float accelScale = kGravity * 8.0f / 32768.0f;
  const float gyroScale = (500.0f / 32768.0f) *
                          3.14159265358979323846f / 180.0f;
  gImuMessage.linear_acceleration.x = ReadInt16(raw) * accelScale;
  gImuMessage.linear_acceleration.y = ReadInt16(raw + 2) * accelScale;
  gImuMessage.linear_acceleration.z = ReadInt16(raw + 4) * accelScale;
  gImuMessage.angular_velocity.x = ReadInt16(raw + 8) * gyroScale - gGyroBias[0];
  gImuMessage.angular_velocity.y = ReadInt16(raw + 10) * gyroScale - gGyroBias[1];
  gImuMessage.angular_velocity.z = ReadInt16(raw + 12) * gyroScale - gGyroBias[2];
  const int64_t nowMs = esp_timer_get_time() / 1000;
  gImuMessage.header.stamp.sec = static_cast<int32_t>(nowMs / 1000);
  gImuMessage.header.stamp.nanosec = static_cast<uint32_t>((nowMs % 1000) * 1000000);
}

void UpdatePid() {
  const int64_t nowUs = esp_timer_get_time();
  const bool timedOut = gLastCommandUs == 0 ||
                        nowUs - gLastCommandUs >
                            static_cast<int64_t>(kCommandTimeoutMs) * 1000;
  for (int i = 0; i < 3; ++i) {
    const float target = timedOut ? 0.0f : gTargetRadPerSec[i];
    const float measured = static_cast<float>(gFeedbackValues[i]);
    const float error = target - measured;
    if (timedOut) {
      gPidIntegral[i] = 0.0f;
      gPidPreviousError[i] = 0.0f;
    } else {
      gPidIntegral[i] = constrain(gPidIntegral[i] + error * kControlPeriodSeconds,
                                  -kPidIntegralLimit, kPidIntegralLimit);
    }
    const float derivative = (error - gPidPreviousError[i]) / kControlPeriodSeconds;
    gPidPreviousError[i] = error;
    gMotorCommandRadPerSec[i] = constrain(
        target + kPidKp * error + kPidKi * gPidIntegral[i] + kPidKd * derivative,
        -kMaxWheelRadPerSec, kMaxWheelRadPerSec);
  }
}

void UpdateEncoderFeedback() {
  constexpr double radiansPerCount =
      2.0 * 3.14159265358979323846 / kEncoderCountsPerRevolution;
  static int64_t previousTimeUs = 0;
  const int64_t nowUs = esp_timer_get_time();
  double elapsedSeconds = kControlPeriodSeconds;
  if (previousTimeUs > 0 && nowUs > previousTimeUs) {
    elapsedSeconds = static_cast<double>(nowUs - previousTimeUs) / 1000000.0;
  }
  previousTimeUs = nowUs;
  if (elapsedSeconds < 0.001) {
    elapsedSeconds = kControlPeriodSeconds;
  }

  for (int i = 0; i < 3; ++i) {
    int count = 0;
    CheckError(pcnt_unit_get_count(gPcntUnits[i], &count),
               "pcnt_unit_get_count");
    CheckError(pcnt_unit_clear_count(gPcntUnits[i]),
               "pcnt_unit_clear_count");
    gFeedbackValues[i] = static_cast<double>(count) * radiansPerCount /
                         elapsedSeconds;
  }
}

void MicroRosTask(void *) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support = {};
  rcl_init_options_t initOptions = rcl_get_zero_initialized_init_options();
  CheckRcl(rcl_init_options_init(&initOptions, allocator), "rcl_init_options_init");

  rmw_init_options_t *rmwOptions = rcl_init_options_get_rmw_init_options(&initOptions);
  CheckRcl(rmw_uros_options_set_udp_address(
      CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmwOptions),
      "rmw_uros_options_set_udp_address");
  CheckRcl(rclc_support_init_with_options(
      &support, 0, nullptr, &initOptions, &allocator),
      "rclc_support_init_with_options");
  CheckRcl(rclc_node_init_default(
      &gNode, "omni_esp32s3_base", "", &support),
      "rclc_node_init_default");

  gCommandMessage.data.data = gCommandValues;
  gCommandMessage.data.size = 3;
  gCommandMessage.data.capacity = 3;
  gFeedbackMessage.data.data = gFeedbackValues;
  gFeedbackMessage.data.size = 3;
  gFeedbackMessage.data.capacity = 3;
  gImuMessage.header.frame_id.data = const_cast<char *>("imu_link");
  gImuMessage.header.frame_id.size = strlen("imu_link");
  gImuMessage.header.frame_id.capacity = gImuMessage.header.frame_id.size + 1;
  gImuMessage.orientation_covariance[0] = -1.0;
  gImuMessage.angular_velocity_covariance[0] = 0.02 * 0.02;
  gImuMessage.angular_velocity_covariance[4] = 0.02 * 0.02;
  gImuMessage.angular_velocity_covariance[8] = 0.02 * 0.02;
  gImuMessage.linear_acceleration_covariance[0] = 0.10 * 0.10;
  gImuMessage.linear_acceleration_covariance[4] = 0.10 * 0.10;
  gImuMessage.linear_acceleration_covariance[8] = 0.10 * 0.10;

  CheckRcl(rclc_subscription_init_default(
      &gCommandSubscription, &gNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/wheel_velocity_cmd"),
      "rclc_subscription_init_default");
  CheckRcl(rclc_publisher_init_default(
      &gFeedbackPublisher, &gNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/wheel_velocity_feedback"),
      "rclc_publisher_init_default");
  CheckRcl(rclc_publisher_init_default(
      &gImuPublisher, &gNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/data_raw"),
      "rclc_publisher_init_default(imu)");
  CheckRcl(rclc_executor_init(&gExecutor, &support.context, 1, &allocator),
           "rclc_executor_init");
  CheckRcl(rclc_executor_add_subscription(
      &gExecutor, &gCommandSubscription, &gCommandMessage,
      &CommandCallback, ON_NEW_DATA),
      "rclc_executor_add_subscription");

  ESP_LOGI(kTag, "micro-ROS node ready; motor standby remains LOW");
  int64_t lastImuPublishUs = 0;
  while (true) {
    UpdateEncoderFeedback();
    UpdatePid();
    UpdateImuMessage();
    CheckRcl(rclc_executor_spin_some(&gExecutor, RCL_MS_TO_NS(1)),
             "rclc_executor_spin_some");
    CheckRcl(rcl_publish(&gFeedbackPublisher, &gFeedbackMessage, nullptr),
             "rcl_publish(feedback)");
    const int64_t nowUs = esp_timer_get_time();
    if (lastImuPublishUs == 0 || nowUs - lastImuPublishUs >= 10000) {
      CheckRcl(rcl_publish(&gImuPublisher, &gImuMessage, nullptr),
               "rcl_publish(imu)");
      lastImuPublishUs = nowUs;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(kTag, "Starting ESP-IDF native migration");
  InitPins();
  InitPwm();
  InitEncoders();
  InitMpu6050();
  ESP_ERROR_CHECK(uros_network_interface_initialize());
  ESP_LOGI(kTag, "Wi-Fi initialized; starting micro-ROS task");
  xTaskCreate(MicroRosTask, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK,
              nullptr, CONFIG_MICRO_ROS_APP_TASK_PRIO, nullptr);
}

// End of file
