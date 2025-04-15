#include "qmi8658_sensor.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace qmi8658
  {

    static const char *const TAG = "qmi8658_sensor";

    void QMI8658SensorComponent::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up QMI8658 Sensors...");
    }

    void QMI8658SensorComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "QMI8658 Sensor:");
      if (this->is_failed())
      {
        ESP_LOGE(TAG, "Communication with QMI8658 failed!");
      }
      LOG_UPDATE_INTERVAL(this);
      LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
      LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
      LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
      LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
      LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
      LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
      LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
    }

    float QMI8658SensorComponent::get_setup_priority() const { return setup_priority::DATA; }

    void QMI8658SensorComponent::update()
    {
      // If we are not yet fully online...
      if (this->qmi8658_->current_state_ != QMI8658_STATE_ONLINE) {
        ESP_LOGD(TAG, "Current State: %d", this->qmi8658_->current_state_);
        // ...try again next time
        return;
      }

      // Read accelerometer values
      float ax = this->read_accel_gyro_data_(this->qmi8658_->accel_x_l_register_, this->qmi8658_->accel_x_h_register_, this->qmi8658_->accel_sensitivity_);
      float ay = this->read_accel_gyro_data_(this->qmi8658_->accel_y_l_register_, this->qmi8658_->accel_y_h_register_, this->qmi8658_->accel_sensitivity_);
      float az = this->read_accel_gyro_data_(this->qmi8658_->accel_z_l_register_, this->qmi8658_->accel_z_h_register_, this->qmi8658_->accel_sensitivity_);

      // Read gyroscope data
      float gx = this->read_accel_gyro_data_(this->qmi8658_->gyro_x_l_register_, this->qmi8658_->gyro_x_h_register_, this->qmi8658_->gyro_sensitivity_);
      float gy = this->read_accel_gyro_data_(this->qmi8658_->gyro_y_l_register_, this->qmi8658_->gyro_y_h_register_, this->qmi8658_->gyro_sensitivity_);
      float gz = this->read_accel_gyro_data_(this->qmi8658_->gyro_z_l_register_, this->qmi8658_->gyro_z_h_register_, this->qmi8658_->gyro_sensitivity_);

      // Publish accelerometer sensor values
      if (this->accel_x_sensor_ != nullptr)
        this->accel_x_sensor_->publish_state(ax);
      if (this->accel_y_sensor_ != nullptr)
        this->accel_y_sensor_->publish_state(ay);
      if (this->accel_z_sensor_ != nullptr)
        this->accel_z_sensor_->publish_state(az);

      // Publish gyroscope sensor values
      if (this->gyro_x_sensor_ != nullptr)
        this->gyro_x_sensor_->publish_state(gx);
      if (this->gyro_y_sensor_ != nullptr)
        this->gyro_y_sensor_->publish_state(gy);
      if (this->gyro_z_sensor_ != nullptr)
        this->gyro_z_sensor_->publish_state(gz);
            
    }

    float QMI8658SensorComponent::read_accel_gyro_data_(i2c::I2CRegister lower_register, i2c::I2CRegister higher_register, float sensitivity) {
      uint8_t data[2];
      data[0] = lower_register.get();
      data[1] = higher_register.get();
      int16_t raw_value = (int16_t)((data[1] << 8) | data[0]);
      return raw_value / sensitivity;
    }

  } // namespace qmi8658
} // namespace esphome
