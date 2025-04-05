#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace qmi8658 {

class QMI8658Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  
  void update() override;

  float get_setup_priority() const override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { this->accel_x_sensor_ = accel_x_sensor; };
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { this->accel_x_sensor_ = accel_y_sensor; };
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { this->accel_x_sensor_ = accel_z_sensor; };
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { this->gyro_x_sensor_ = gyro_x_sensor; };
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { this->gyro_x_sensor_ = gyro_y_sensor; };
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { this->gyro_x_sensor_ = gyro_z_sensor; };

 protected:
  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
};
;

}  // namespace qmi8658
}  // namespace esphome