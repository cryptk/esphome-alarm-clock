#include "qmi8658.h" // Include the header we'll create below
#include "esphome/core/log.h"

namespace esphome
{
  namespace qmi8658
  {

    static const char *const TAG = "qmi8658";

    // Datasheet: https://www.qstcorp.com/upload/pdf/202301/13-52-25%20QMI8658A%20Datasheet%20Rev%20A.pdf

    // General Purpose Registers
    static constexpr uint8_t QMI8658_REG_WHO_AM_I = 0x00;    // Device Identifier
    static constexpr uint8_t QMI8658_REG_REVISION_ID = 0x01; // Device Revision ID

    // Setup and Control Registers
    static constexpr uint8_t QMI8658_REG_CTRL1 = 0x02; // SPI Interface and Sensor Enable
    static constexpr uint8_t QMI8658_REG_CTRL2 = 0x03; // Accelerometer: Output Data Rate, Full Scale, Self Test
    static constexpr uint8_t QMI8658_REG_CTRL3 = 0x04; // Gyroscope: Output Data Rate, Full Scale, Self Test
    static constexpr uint8_t QMI8658_REG_CTRL4 = 0x05; // Reserved
    static constexpr uint8_t QMI8658_REG_CTRL5 = 0x06; // Low pass filter setting.
    static constexpr uint8_t QMI8658_REG_CTRL6 = 0x07; // Reserved
    static constexpr uint8_t QMI8658_REG_CTRL7 = 0x08; // Enable Sensors
    static constexpr uint8_t QMI8658_REG_CTRL8 = 0x09; // Motion Detection Control
    static constexpr uint8_t QMI8658_REG_CTRL9 = 0x0A; // Host Commands

    // Host Controlled Calibration Registers (See CTRL9 in datasheet, Usage is Optional)
    static constexpr uint8_t QMI8658_REG_CAL1_L = 0x0B; // Calibration Register 1 lower 8 bits
    static constexpr uint8_t QMI8658_REG_CAL1_H = 0x0C; // Calibration Register 1 upper 8 bits
    static constexpr uint8_t QMI8658_REG_CAL2_L = 0x0D; // Calibration Register 2 lower 8 bits
    static constexpr uint8_t QMI8658_REG_CAL2_H = 0x0E; // Calibration Register 2 upper 8 bits
    static constexpr uint8_t QMI8658_REG_CAL3_L = 0x0F; // Calibration Register 3 lower 8 bits
    static constexpr uint8_t QMI8658_REG_CAL3_H = 0x10; // Calibration Register 3 upper 8 bits
    static constexpr uint8_t QMI8658_REG_CAL4_L = 0x11; // Calibration Register 4 lower 8 bits
    static constexpr uint8_t QMI8658_REG_CAL4_H = 0x12; // Calibration Register 4 upper 8 bits

    // FIFO Registers
    static constexpr uint8_t QMI8658_REG_FIFO_WTM_TH = 0x13;   // FIFO watermark level, in ODRs
    static constexpr uint8_t QMI8658_REG_FIFO_CTRL = 0x14;     // FIFO Setup
    static constexpr uint8_t QMI8658_REG_FIFO_SMPL_CNT = 0x15; // FIFO sample count LSBs
    static constexpr uint8_t QMI8658_REG_FIFO_STATUS = 0x16;   // FIFO Status
    static constexpr uint8_t QMI8658_REG_FIFO_DATA = 0x17;     // FIFO Data

    // Status Registers
    static constexpr uint8_t QMI8658_REG_STATUSINT = 0x2D; // Sensor Data Availability with the Locking mechanism, CmdDone (CTRL9 protocol bit).
    static constexpr uint8_t QMI8658_REG_STATUS0 = 0x2E;   // Output Data Over Run and Data Availability.
    static constexpr uint8_t QMI8658_REG_STATUS1 = 0x2F;   // Miscellaneous Status: Any Motion, No Motion, Significant Motion, Pedometer, Tap.

    // Timestamp Register
    static constexpr uint8_t QMI8658_REG_TIMESTAMP_LOW = 0x30;  // Timestamp Lower 8 bits
    static constexpr uint8_t QMI8658_REG_TIMESTAMP_MID = 0x31;  // Timestamp middle 8 bits
    static constexpr uint8_t QMI8658_REG_TIMESTAMP_HIGH = 0x32; // Timestamp upper 8 bits;

    // Data Output Registers (16 bits 2’s Complement Except COD Sensor Data)
    static constexpr uint8_t QMI8658_REG_TEMP_L = 0x33; // Temperature Output Data lower 8 bits
    static constexpr uint8_t QMI8658_REG_TEMP_H = 0x34; // Temperature Output Data upper 8 bits

    static constexpr uint8_t QMI8658_REG_AX_L = 0x35; // X-axis Acceleration lower 8 bits
    static constexpr uint8_t QMI8658_REG_AX_H = 0x36; // X-axis Acceleration upper 8 bits
    static constexpr uint8_t QMI8658_REG_AY_L = 0x37; // Y-axis Acceleration lower 8 bits
    static constexpr uint8_t QMI8658_REG_AY_H = 0x38; // Y-axis Acceleration upper 8 bits
    static constexpr uint8_t QMI8658_REG_AZ_L = 0x39; // Z-axis Acceleration lower 8 bits
    static constexpr uint8_t QMI8658_REG_AZ_H = 0x3A; // Z-axis Acceleration upper 8 bits

    static constexpr uint8_t QMI8658_REG_GX_L = 0x3B; // X-axis Angular Rate lower 8 bits
    static constexpr uint8_t QMI8658_REG_GX_H = 0x3C; // X-axis Angular Rate upper 8 bits
    static constexpr uint8_t QMI8658_REG_GY_L = 0x3D; // Y-axis Angular Rate lower 8 bits
    static constexpr uint8_t QMI8658_REG_GY_H = 0x3E; // Y-axis Angular Rate upper 8 bits
    static constexpr uint8_t QMI8658_REG_GZ_L = 0x3F; // Z-axis Angular Rate lower 8 bits
    static constexpr uint8_t QMI8658_REG_GZ_H = 0x40; // Z-axis Angular Rate upper 8 bits

    // COD Indication and General Purpose Registers
    static constexpr uint8_t QMI8658_REG_COD_STATUS = 0x46; // Calibration-On-Demand status register

    static constexpr uint8_t QMI8658_REG_dQW_L = 0x49; // General purpose register
    static constexpr uint8_t QMI8658_REG_dQW_H = 0x4A; // General purpose register
    static constexpr uint8_t QMI8658_REG_dQX_L = 0x4B; // General purpose register
    static constexpr uint8_t QMI8658_REG_dQX_H = 0x4C; // Reserved
    static constexpr uint8_t QMI8658_REG_dQY_L = 0x4D; // General purpose register
    static constexpr uint8_t QMI8658_REG_dQY_H = 0x4E; // Reserved
    static constexpr uint8_t QMI8658_REG_dQZ_L = 0x4F; // Reserved
    static constexpr uint8_t QMI8658_REG_dQZ_H = 0x50; // Reserved

    static constexpr uint8_t QMI8658_REG_dVX_L = 0x51; // General purpose register
    static constexpr uint8_t QMI8658_REG_dVX_H = 0x52; // General purpose register
    static constexpr uint8_t QMI8658_REG_dVY_L = 0x53; // General purpose register
    static constexpr uint8_t QMI8658_REG_dVY_H = 0x54; // General purpose register
    static constexpr uint8_t QMI8658_REG_dVZ_L = 0x55; // General purpose register
    static constexpr uint8_t QMI8658_REG_dVZ_H = 0x56; // General purpose register

    static constexpr uint8_t QMI8658_REG_TAP_STATUS = 0x59;    // Axis, direction, number of detected Tap
    static constexpr uint8_t QMI8658_REG_STEP_CNT_LOW = 0x5A;  // Low byte of step count of Pedometer
    static constexpr uint8_t QMI8658_REG_STEP_CNT_MIDL = 0x5B; // Middle byte of step count of Pedometer
    static constexpr uint8_t QMI8658_REG_STEP_CNT_HIGH = 0x5C; // High byte of step count of Pedometer

    // Reset Register
    static constexpr uint8_t QMI8658_REG_RESET = 0x60; // Soft Reset Register

    typedef enum
    {
      ACC_SELF_TEST_DISENABLE = 0,
      ACC_SELF_TEST_ENABLE,
    } qmi8658_acc_self_test_t;

    typedef enum
    {
      ACC_SCALE_2G = 0,
      ACC_SCALE_4G,
      ACC_SCALE_8G,
      ACC_SCALE_16G,
    } qmi8658_acc_scale_t;

    typedef enum
    {
      ACC_OUTPUT_DATA_RATE_1000HZ = 3,
      ACC_OUTPUT_DATA_RATE_500HZ,
      ACC_OUTPUT_DATA_RATE_250HZ,
      ACC_OUTPUT_DATA_RATE_125HZ,
      ACC_OUTPUT_DATA_RATE_62_5HZ,
      ACC_OUTPUT_DATA_RATE_31_25HZ,
    } qmi8658_acc_odr_t;

    typedef enum
    {
      GYRO_SELF_TEST_DISENABLE = 0,
      GYRO_SELF_TEST_ENABLE,
    } qmi8658_gyro_self_test_t;

    typedef enum
    {
      GYRO_SCALE_16DPS = 0,
      GYRO_SCALE_32DPS,
      GYRO_SCALE_64DPS,
      GYRO_SCALE_128DPS,
      GYRO_SCALE_256DPS,
      GYRO_SCALE_512DPS,
      GYRO_SCALE_1024DPS,
      GYRO_SCALE_2048DPS,
    } qmi8658_gyro_scale_t;

    typedef enum
    {
      GYRO_OUTPUT_DATA_RATE_7174_4 = 0,
      GYRO_OUTPUT_DATA_RATE_3584_2,
      GYRO_OUTPUT_DATA_RATE_1793_6,
      GYRO_OUTPUT_DATA_RATE_896_8,
      GYRO_OUTPUT_DATA_RATE_448_4,
      GYRO_OUTPUT_DATA_RATE_224_2,
      GYRO_OUTPUT_DATA_RATE_112_1,
      GYRO_OUTPUT_DATA_RATE_56_05,
      GYRO_OUTPUT_DATA_RATE_28_025,
    } qmi8658_gyro_odr_t;

    // const float GRAVITY_EARTH = 9.80665f;

    // Utility function to combine two unsigned 8-bit registers into a signed 16-bit value using twos compliment
    int16_t combine_bytes(uint8_t low, uint8_t high)
    {
      return (int16_t)((high << 8) | low);
    }

    void QMI8658Component::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up QMI8658...");

      // Ask the chip for an identifier.  0x05 means it's a QST sensor.
      uint8_t who_am_i;
      if (!this->read_byte(QMI8658_REG_WHO_AM_I, &who_am_i) || (who_am_i != 0x05))
      {
        this->mark_failed();
        return;
      }
      uint8_t revision_id;
      if (!this->read_byte(QMI8658_REG_REVISION_ID, &revision_id))
      {
        this->mark_failed();
        return;
      }
      ESP_LOGCONFIG(TAG, "Device revision: %d", revision_id);

      uint8_t ctrl1;
      // Read the existing CTRL1 register
      this->read_byte(QMI8658_REG_CTRL1, &ctrl1);
      // Enable the internal 2MHz oscillator
      ctrl1 &= 0xFE; // ensure bit 1 is 0
      // Enable automatic address incrementation for faster block reads
      ctrl1 |= 0x40; // ensure bit 6 is 1
      this->write_byte(QMI8658_REG_CTRL1, ctrl1);

      // Set Acceleromenter Output Data Rate to 1000Hz, full-scale +/- 2g
      this->write_byte(QMI8658_REG_CTRL2, 0x03);
      // Set Gyro Output Data Rate to 896.8Hz, Full-scale +/- 2048dps
      this->write_byte(QMI8658_REG_CTRL3, 0x73);
      // Enable Accelerometer (bit 1) and Gyroscope (bit 2)
      this->write_byte(QMI8658_REG_CTRL7, 0x03);

      // // Initialization code
      // // Example: Disable all sensors initially
      // this->write_byte(QMI8658_REG_CTRL7, 0x00);  // Assuming 0x00 disables all
      // // Add other initialization routines here (e.g., setting up accelerometer/gyroscope ranges, etc.)
      // // 0x4D can be overwritten later as it is also the QMI8658_REG_dQY_L register, so it shoudl be checked somewhat quickly
    }

    void QMI8658Component::dump_config()
    {
      ESP_LOGCONFIG(TAG, "QMI8658:");
      LOG_I2C_DEVICE(this);
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

    void QMI8658Component::update()
    {
      // Read accelerometer and gyroscope data
      uint16_t raw_data[12]; // 6 bytes for accelerometer, 6 bytes for gyroscope

      uint8_t high_bits;
      uint8_t low_bits;

      // Read accelerometer data X
      this->read_byte(QMI8658_REG_AX_L, &low_bits);
      this->read_byte(QMI8658_REG_AX_H, &high_bits);
      int16_t ax = combine_bytes(low_bits, high_bits);

      // Read accelerometer data Y
      this->read_byte(QMI8658_REG_AY_L, &low_bits);
      this->read_byte(QMI8658_REG_AY_H, &high_bits);
      int16_t ay = combine_bytes(low_bits, high_bits);

      // Read accelerometer data Z
      this->read_byte(QMI8658_REG_AZ_L, &low_bits);
      this->read_byte(QMI8658_REG_AZ_H, &high_bits);
      int16_t az = combine_bytes(low_bits, high_bits);

      // Read gyroscope data (X, Y, Z)
      this->read_byte(QMI8658_REG_GX_L, &low_bits);
      this->read_byte(QMI8658_REG_GX_H, &high_bits);
      int16_t gx = combine_bytes(low_bits, high_bits);

      this->read_byte(QMI8658_REG_GY_L, &low_bits);
      this->read_byte(QMI8658_REG_GY_H, &high_bits);
      int16_t gy = combine_bytes(low_bits, high_bits);

      this->read_byte(QMI8658_REG_GZ_L, &low_bits);
      this->read_byte(QMI8658_REG_GZ_H, &high_bits);
      int16_t gz = combine_bytes(low_bits, high_bits);

      // Convert raw data to physical units (g's and dps) -  **Need sensitivity values from datasheet**
      //  Example:  (replace with actual sensitivity values)
      float ax_g = ax / 16384.0f; //  +/- 2g range (see Table 7 in datasheet)
      float ay_g = ay / 16384.0f;
      float az_g = az / 16384.0f;
      float gx_dps = gx / 16.0f; // +/- 2048 dps range (see Table 8 in datasheet)
      float gy_dps = gy / 16.0f;
      float gz_dps = gz / 16.0f;

      // Publish accelerometer sensor values
      if (this->accel_x_sensor_ != nullptr)
        this->accel_x_sensor_->publish_state(ax_g);
      if (this->accel_y_sensor_ != nullptr)
        this->accel_y_sensor_->publish_state(ay_g);
      if (this->accel_z_sensor_ != nullptr)
        this->accel_z_sensor_->publish_state(az_g);

      // Publish gyroscope sensor values
      if (this->gyro_x_sensor_ != nullptr)
        this->gyro_x_sensor_->publish_state(gx_dps);
      if (this->gyro_y_sensor_ != nullptr)
        this->gyro_y_sensor_->publish_state(gy_dps);
      if (this->gyro_z_sensor_ != nullptr)
        this->gyro_z_sensor_->publish_state(gz_dps);
    }

    float QMI8658Component::get_setup_priority() const { return setup_priority::DATA; }

  } // namespace qmi8658
} // namespace esphome