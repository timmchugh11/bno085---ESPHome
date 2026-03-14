#pragma once

#ifdef USE_ARDUINO
#if __has_include(<Adafruit_BNO08x.h>)
#include <Adafruit_BNO08x.h>
#elif __has_include("Adafruit_BNO08x/src/Adafruit_BNO08x.h")
#include "Adafruit_BNO08x/src/Adafruit_BNO08x.h"
#elif __has_include("Adafruit_BNO08x.h")
#include "Adafruit_BNO08x.h"
#else
#error "Adafruit_BNO08x header not found. Install adafruit/Adafruit BNO08x or vendor it locally."
#endif
#endif

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
#ifdef USE_ARDUINO
  void handle_sensor_event_(const sh2_SensorValue_t &value);
  void publish_rotation_vector_(const sh2_SensorValue_t &value);
  void publish_accelerometer_(const sh2_SensorValue_t &value);
  void publish_gyroscope_(const sh2_SensorValue_t &value);

  Adafruit_BNO08x bno08x_ = Adafruit_BNO08x(-1);
#endif
  bool initialized_{false};

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
