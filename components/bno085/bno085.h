#pragma once

#include "sh2.h"
#include "sh2_SensorValue.h"

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace bno085 {

class BNO085Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_roll_sensor(sensor::Sensor *sensor) { this->roll_sensor_ = sensor; }
  void set_pitch_sensor(sensor::Sensor *sensor) { this->pitch_sensor_ = sensor; }
  void set_yaw_sensor(sensor::Sensor *sensor) { this->yaw_sensor_ = sensor; }
  void set_accel_x_sensor(sensor::Sensor *sensor) { this->accel_x_sensor_ = sensor; }
  void set_accel_y_sensor(sensor::Sensor *sensor) { this->accel_y_sensor_ = sensor; }
  void set_accel_z_sensor(sensor::Sensor *sensor) { this->accel_z_sensor_ = sensor; }
  void set_gyro_x_sensor(sensor::Sensor *sensor) { this->gyro_x_sensor_ = sensor; }
  void set_gyro_y_sensor(sensor::Sensor *sensor) { this->gyro_y_sensor_ = sensor; }
  void set_gyro_z_sensor(sensor::Sensor *sensor) { this->gyro_z_sensor_ = sensor; }

  void set_rotation_vector_interval_ms(uint32_t interval) {
    this->rotation_vector_interval_ms_ = interval;
  }
  void set_accelerometer_interval_ms(uint32_t interval) {
    this->accelerometer_interval_ms_ = interval;
  }
  void set_gyroscope_interval_ms(uint32_t interval) {
    this->gyroscope_interval_ms_ = interval;
  }

 protected:
  void enable_reports_();
  void handle_sensor_event_(const sh2_SensorValue_t &value);
  void publish_rotation_vector_(const sh2_SensorValue_t &value);
  void publish_accelerometer_(const sh2_SensorValue_t &value);
  void publish_gyroscope_(const sh2_SensorValue_t &value);

  static int hal_open_(sh2_Hal_t *self);
  static void hal_close_(sh2_Hal_t *self);
  static int hal_read_(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *timestamp_us);
  static int hal_write_(sh2_Hal_t *self, uint8_t *buffer, unsigned len);
  static uint32_t hal_get_time_us_(sh2_Hal_t *self);

  static void async_event_callback_(void *cookie, sh2_AsyncEvent_t *event);
  static void sensor_event_callback_(void *cookie, sh2_SensorEvent_t *event);

  void on_async_event_(const sh2_AsyncEvent_t &event);
  void on_sensor_event_(const sh2_SensorEvent_t &event);

  static BNO085Component *active_instance_;

  sh2_Hal_t hal_{};
  bool initialized_{false};
  bool reset_occurred_{false};

  static constexpr uint16_t I2C_CHUNK_SIZE = 32;
  uint8_t i2c_chunk_buffer_[I2C_CHUNK_SIZE]{};

  uint32_t rotation_vector_interval_ms_{50};
  uint32_t accelerometer_interval_ms_{50};
  uint32_t gyroscope_interval_ms_{50};

  sensor::Sensor *roll_sensor_{nullptr};
  sensor::Sensor *pitch_sensor_{nullptr};
  sensor::Sensor *yaw_sensor_{nullptr};

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};

  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
};

}  // namespace bno085
}  // namespace esphome
