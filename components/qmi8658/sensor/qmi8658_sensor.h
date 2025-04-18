#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "../qmi8658.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace qmi8658 {

class QMI8658SensorComponent : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;

  void update() override;

  float get_setup_priority() const override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; };
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; };
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; };
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; };
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; };
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; };
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

  void set_parent(QMI8658Component *qmi8658) { this->qmi8658_ = qmi8658; };

 protected:
  QMI8658Component *qmi8658_{nullptr};

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};

  sensor::Sensor *temperature_sensor_{nullptr};

  float read_accel_gyro_data_(i2c::I2CRegister lower_register, i2c::I2CRegister higher_register, float sensitivity);
};
;

}  // namespace qmi8658
}  // namespace esphome
