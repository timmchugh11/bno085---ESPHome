#include "bno085.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#ifdef USE_ESP32
#include <esp_timer.h>
#endif

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bno085 {

static const char *const TAG = "bno085";
static constexpr float RAD_TO_DEG = 57.29577951308232f;

BNO085Component *BNO085Component::active_instance_ = nullptr;

void BNO085Component::setup() {
  active_instance_ = this;

  this->hal_.open = &BNO085Component::hal_open_;
  this->hal_.close = &BNO085Component::hal_close_;
  this->hal_.read = &BNO085Component::hal_read_;
  this->hal_.write = &BNO085Component::hal_write_;
  this->hal_.getTimeUs = &BNO085Component::hal_get_time_us_;

  int status = sh2_open(&this->hal_, &BNO085Component::async_event_callback_, this);
  if (status != SH2_OK) {
    ESP_LOGE(TAG, "Failed to open SH2 session, status=%d", status);
    this->mark_failed();
    return;
  }

  sh2_ProductIds_t prod_ids;
  std::memset(&prod_ids, 0, sizeof(prod_ids));
  status = sh2_getProdIds(&prod_ids);
  if (status != SH2_OK) {
    ESP_LOGE(TAG, "Failed to read product IDs, status=%d", status);
    sh2_close();
    this->mark_failed();
    return;
  }

  status = sh2_setSensorCallback(&BNO085Component::sensor_event_callback_, this);
  if (status != SH2_OK) {
    ESP_LOGE(TAG, "Failed to register sensor callback, status=%d", status);
    sh2_close();
    this->mark_failed();
    return;
  }

  this->initialized_ = true;
  this->enable_reports_();
  ESP_LOGI(TAG, "BNO085 initialized using native SH2 HAL");
}

void BNO085Component::update() {
  if (!this->initialized_) {
    return;
  }

  if (this->reset_occurred_) {
    this->reset_occurred_ = false;
    ESP_LOGW(TAG, "BNO085 reset detected, re-enabling reports");
    this->enable_reports_();
  }

  // Service SH2 repeatedly to drain any pending packets from the transport.
  for (uint8_t i = 0; i < 6; i++) {
    sh2_service();
  }
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

  sh2_SensorConfig_t config{};
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  const uint32_t rotation_us = this->rotation_vector_interval_ms_ * 1000UL;
  const uint32_t accel_us = this->accelerometer_interval_ms_ * 1000UL;
  const uint32_t gyro_us = this->gyroscope_interval_ms_ * 1000UL;

  if ((this->roll_sensor_ != nullptr) || (this->pitch_sensor_ != nullptr) ||
      (this->yaw_sensor_ != nullptr)) {
    config.reportInterval_us = rotation_us;
    if (sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config) != SH2_OK) {
      ESP_LOGW(TAG, "Failed to configure SH2_ROTATION_VECTOR");
    }
  }

  if ((this->accel_x_sensor_ != nullptr) || (this->accel_y_sensor_ != nullptr) ||
      (this->accel_z_sensor_ != nullptr)) {
    config.reportInterval_us = accel_us;
    if (sh2_setSensorConfig(SH2_ACCELEROMETER, &config) != SH2_OK) {
      ESP_LOGW(TAG, "Failed to configure SH2_ACCELEROMETER");
    }
  }

  if ((this->gyro_x_sensor_ != nullptr) || (this->gyro_y_sensor_ != nullptr) ||
      (this->gyro_z_sensor_ != nullptr)) {
    config.reportInterval_us = gyro_us;
    if (sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config) != SH2_OK) {
      ESP_LOGW(TAG, "Failed to configure SH2_GYROSCOPE_CALIBRATED");
    }
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

int BNO085Component::hal_open_(sh2_Hal_t *self) {
  (void) self;
  auto *inst = active_instance_;
  if (inst == nullptr) {
    return -1;
  }

  const uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (inst->write(softreset_pkt, sizeof(softreset_pkt)) == i2c::ERROR_OK) {
      success = true;
      break;
    }
    delay_microseconds_safe(30000);
  }

  if (!success) {
    return -1;
  }

  delay_microseconds_safe(300000);
  return 0;
}

void BNO085Component::hal_close_(sh2_Hal_t *self) { (void) self; }

int BNO085Component::hal_read_(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *timestamp_us) {
  (void) self;
  auto *inst = active_instance_;
  if (inst == nullptr || buffer == nullptr || len == 0) {
    return 0;
  }

  if (timestamp_us != nullptr) {
    *timestamp_us = hal_get_time_us_(self);
  }

  uint8_t header[4];
  if (inst->read(header, sizeof(header)) != i2c::ERROR_OK) {
    return 0;
  }

  uint16_t packet_size = static_cast<uint16_t>(header[0]) | (static_cast<uint16_t>(header[1]) << 8);
  packet_size &= static_cast<uint16_t>(~0x8000);

  if (packet_size == 0 || packet_size > len) {
    return 0;
  }

  uint16_t cargo_remaining = packet_size;
  bool first_read = true;

  while (cargo_remaining > 0) {
    uint16_t read_size;
    if (first_read) {
      read_size = std::min<uint16_t>(I2C_CHUNK_SIZE, cargo_remaining);
    } else {
      read_size = std::min<uint16_t>(I2C_CHUNK_SIZE, static_cast<uint16_t>(cargo_remaining + 4));
    }

    if (inst->read(inst->i2c_chunk_buffer_, read_size) != i2c::ERROR_OK) {
      return 0;
    }

    uint16_t copied;
    if (first_read) {
      copied = read_size;
      std::memcpy(buffer, inst->i2c_chunk_buffer_, copied);
      first_read = false;
    } else {
      copied = read_size - 4;
      std::memcpy(buffer, inst->i2c_chunk_buffer_ + 4, copied);
    }

    buffer += copied;
    cargo_remaining -= copied;
  }

  return static_cast<int>(packet_size);
}

int BNO085Component::hal_write_(sh2_Hal_t *self, uint8_t *buffer, unsigned len) {
  (void) self;
  auto *inst = active_instance_;
  if (inst == nullptr || buffer == nullptr || len == 0) {
    return 0;
  }

  const uint16_t write_size = std::min<uint16_t>(I2C_CHUNK_SIZE, static_cast<uint16_t>(len));
  if (inst->write(buffer, write_size) != i2c::ERROR_OK) {
    return 0;
  }

  return static_cast<int>(write_size);
}

uint32_t BNO085Component::hal_get_time_us_(sh2_Hal_t *self) {
  (void) self;
#ifdef USE_ESP32
  return static_cast<uint32_t>(esp_timer_get_time());
#else
  return 0;
#endif
}

void BNO085Component::async_event_callback_(void *cookie, sh2_AsyncEvent_t *event) {
  if (cookie == nullptr || event == nullptr) {
    return;
  }
  static_cast<BNO085Component *>(cookie)->on_async_event_(*event);
}

void BNO085Component::sensor_event_callback_(void *cookie, sh2_SensorEvent_t *event) {
  if (cookie == nullptr || event == nullptr) {
    return;
  }
  static_cast<BNO085Component *>(cookie)->on_sensor_event_(*event);
}

void BNO085Component::on_async_event_(const sh2_AsyncEvent_t &event) {
  if (event.eventId == SH2_RESET) {
    this->reset_occurred_ = true;
  }
}

void BNO085Component::on_sensor_event_(const sh2_SensorEvent_t &event) {
  sh2_SensorValue_t value{};
  const int rc = sh2_decodeSensorEvent(&value, &event);
  if (rc != SH2_OK) {
    return;
  }

  this->handle_sensor_event_(value);
}

}  // namespace bno085
}  // namespace esphome
