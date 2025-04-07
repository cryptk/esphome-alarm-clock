#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome
{
  namespace qmi8658
  {

    class QMI8658Component : public PollingComponent, public i2c::I2CDevice
    {
    public:
      void setup() override;
      void dump_config() override;

      void update() override;

      float get_setup_priority() const override;

      void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; };
      void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; };
      void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; };
      void set_accel_output_data_rate(uint16_t accel_output_data_rate) { accel_output_data_rate_ = accel_output_data_rate; };
      void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; };
      void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; };
      void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; };
      void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

      void configure_accel_output_data_rate();

    protected:
      sensor::Sensor *accel_x_sensor_{nullptr};
      sensor::Sensor *accel_y_sensor_{nullptr};
      sensor::Sensor *accel_z_sensor_{nullptr};
      sensor::Sensor *gyro_x_sensor_{nullptr};
      sensor::Sensor *gyro_y_sensor_{nullptr};
      sensor::Sensor *gyro_z_sensor_{nullptr};
      sensor::Sensor *temperature_sensor_{nullptr};

      void set_register_value(uint8_t register_address, uint8_t value, uint8_t offset);

      uint16_t accel_output_data_rate_{};
    };
    ;

  } // namespace qmi8658
} // namespace esphome