#include "qmi8658.h"  // Include the header we'll create below
#include "esphome/core/log.h"

namespace esphome {
namespace qmi8658 {

static const char *const TAG = "qmi8658";

// Constants from the QMI8658C datasheet
const uint8_t QMI8658_I2C_ADDRESS = 0x6A; // Default I2C address (SA0 low)
const uint8_t QMI8658_REG_CTRL7 = 0x07;   // Control register for enabling sensors
const uint8_t QMI8658_REG_AX_L = 0x35;    // Register address for accelerometer X-axis LSB
const uint8_t QMI8658_REG_GX_L = 0x3B;    // Register address for gyroscope X-axis LSB

const float GRAVITY_EARTH = 9.80665f;
// ... other relevant register addresses

// Utility function to combine two 8-bit registers into a 16-bit value
int16_t combine_bytes(uint8_t low, uint8_t high) {
  return (int16_t)((high << 8) | low);
}

void QMI8658Component::setup() {
  // Initialization code
  // Example: Disable all sensors initially
  this->write_byte(QMI8658_REG_CTRL7, 0x00);  // Assuming 0x00 disables all
  // Add other initialization routines here (e.g., setting up accelerometer/gyroscope ranges, etc.)
}

void MPU6050Component::dump_config() {
  ESP_LOGCONFIG(TAG, "QMI8658:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with QMI8658 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  // LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void QMI8658Component::update() {
  // Read accelerometer and gyroscope data
  uint8_t raw_data[12];  // 6 bytes for accelerometer, 6 bytes for gyroscope
  
  // Read accelerometer data (X, Y, Z)
  this->read_bytes_16(QMI8658_REG_AX_L, raw_data, 6);
  int16_t ax = combine_bytes(raw_data[0], raw_data[1]);
  int16_t ay = combine_bytes(raw_data[2], raw_data[3]);
  int16_t az = combine_bytes(raw_data[4], raw_data[5]);
  
  // Read gyroscope data (X, Y, Z)
  this->read_bytes_16(QMI8658_REG_GX_L, raw_data + 6, 6); // Offset in buffer
  int16_t gx = combine_bytes(raw_data[0 + 6], raw_data[1 + 6]);
  int16_t gy = combine_bytes(raw_data[2 + 6], raw_data[3 + 6]);
  int16_t gz = combine_bytes(raw_data[4 + 6], raw_data[5 + 6]);
  
  // Convert raw data to physical units (g's and dps) -  **Need sensitivity values from datasheet**
  //  Example:  (replace with actual sensitivity values)
  float ax_g = ax / 16384.0f;  //  +/- 2g range (see Table 7 in datasheet)
  float ay_g = ay / 16384.0f;
  float az_g = az / 16384.0f;
  float gx_dps = gx / 16.0f;   // +/- 2048 dps range (see Table 8 in datasheet)
  float gy_dps = gy / 16.0f;
  float gz_dps = gz / 16.0f;
  
  // Publish sensor values
  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(ax_g);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(ay_g);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(az_g);

  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gx_dps);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gy_dps);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gz_dps);
}

float QMI8658Component::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace qmi8658
}  // namespace esphome