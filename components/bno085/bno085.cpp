#include "bno085.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace bno085 {

static const char *const TAG = "bno085";
static constexpr float RAD_TO_DEG = 57.29577951308232f;

void BNO085Component::setup() {
#ifdef USE_ARDUINO
  if (!this->bno08x_.begin_I2C(this->address_)) {
    ESP_LOGE(TAG, "Failed to initialize BNO085 at address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  this->initialized_ = true;
  this->enable_reports_();
  ESP_LOGI(TAG, "BNO085 initialized");
#else
  ESP_LOGE(TAG, "BNO085 component currently requires Arduino framework");
  this->mark_failed();
#endif
}

void BNO085Component::update() {
#ifdef USE_ARDUINO
  if (!this->initialized_) {
    return;
  }

  if (this->bno08x_.wasReset()) {
    ESP_LOGW(TAG, "BNO085 reset detected, re-enabling reports");
    this->enable_reports_();
  }

  sh2_SensorValue_t sensor_value;
  while (this->bno08x_.getSensorEvent(&sensor_value)) {
    this->handle_sensor_event_(sensor_value);
  }
#endif
}

void BNO085Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BNO085:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Roll", this->roll_sensor_);
  LOG_SENSOR("  ", "Pitch", this->pitch_sensor_);
  LOG_SENSOR("  ", "Yaw", this->yaw_sensor_);
  LOG_SENSOR("  ", "Accel X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Accel Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Accel Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
}

void BNO085Component::enable_reports_() {
  if (!this->initialized_) {
    return;
  }

  const uint32_t rotation_us = this->rotation_vector_interval_ms_ * 1000UL;
  const uint32_t accel_us = this->accelerometer_interval_ms_ * 1000UL;
  const uint32_t gyro_us = this->gyroscope_interval_ms_ * 1000UL;

  if ((this->roll_sensor_ != nullptr) || (this->pitch_sensor_ != nullptr) ||
      (this->yaw_sensor_ != nullptr)) {
    this->bno08x_.enableReport(SH2_ROTATION_VECTOR, rotation_us);
  }

  if ((this->accel_x_sensor_ != nullptr) || (this->accel_y_sensor_ != nullptr) ||
      (this->accel_z_sensor_ != nullptr)) {
    this->bno08x_.enableReport(SH2_ACCELEROMETER, accel_us);
  }

  if ((this->gyro_x_sensor_ != nullptr) || (this->gyro_y_sensor_ != nullptr) ||
      (this->gyro_z_sensor_ != nullptr)) {
    this->bno08x_.enableReport(SH2_GYROSCOPE_CALIBRATED, gyro_us);
  }
}

void BNO085Component::handle_sensor_event_(const sh2_SensorValue_t &value) {
  switch (value.sensorId) {
    case SH2_ROTATION_VECTOR:
      this->publish_rotation_vector_(value);
      break;
    case SH2_ACCELEROMETER:
      this->publish_accelerometer_(value);
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      this->publish_gyroscope_(value);
      break;
    default:
      break;
  }
}

void BNO085Component::publish_rotation_vector_(const sh2_SensorValue_t &value) {
  const float qw = value.un.rotationVector.real;
  const float qx = value.un.rotationVector.i;
  const float qy = value.un.rotationVector.j;
  const float qz = value.un.rotationVector.k;

  const float sinr_cosp = 2.0f * ((qw * qx) + (qy * qz));
  const float cosr_cosp = 1.0f - (2.0f * ((qx * qx) + (qy * qy)));
  const float roll = std::atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

  float sinp = 2.0f * ((qw * qy) - (qz * qx));
  sinp = std::max(-1.0f, std::min(1.0f, sinp));
  const float pitch = std::asin(sinp) * RAD_TO_DEG;

  const float siny_cosp = 2.0f * ((qw * qz) + (qx * qy));
  const float cosy_cosp = 1.0f - (2.0f * ((qy * qy) + (qz * qz)));
  const float yaw = std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;

  if (this->roll_sensor_ != nullptr) {
    this->roll_sensor_->publish_state(roll);
  }
  if (this->pitch_sensor_ != nullptr) {
    this->pitch_sensor_->publish_state(pitch);
  }
  if (this->yaw_sensor_ != nullptr) {
    this->yaw_sensor_->publish_state(yaw);
  }
}

void BNO085Component::publish_accelerometer_(const sh2_SensorValue_t &value) {
  if (this->accel_x_sensor_ != nullptr) {
    this->accel_x_sensor_->publish_state(value.un.accelerometer.x);
  }
  if (this->accel_y_sensor_ != nullptr) {
    this->accel_y_sensor_->publish_state(value.un.accelerometer.y);
  }
  if (this->accel_z_sensor_ != nullptr) {
    this->accel_z_sensor_->publish_state(value.un.accelerometer.z);
  }
}

void BNO085Component::publish_gyroscope_(const sh2_SensorValue_t &value) {
  if (this->gyro_x_sensor_ != nullptr) {
    this->gyro_x_sensor_->publish_state(value.un.gyroscope.x * RAD_TO_DEG);
  }
  if (this->gyro_y_sensor_ != nullptr) {
    this->gyro_y_sensor_->publish_state(value.un.gyroscope.y * RAD_TO_DEG);
  }
  if (this->gyro_z_sensor_ != nullptr) {
    this->gyro_z_sensor_->publish_state(value.un.gyroscope.z * RAD_TO_DEG);
  }
}

}  // namespace bno085
}  // namespace esphome
