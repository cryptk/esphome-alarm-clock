#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome
{
  namespace qmi8658
  {

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
    static constexpr uint8_t QMI8658_REG_dQY_L = 0x4D; // General purpose register, contains reset status
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
      QMI8658_STATE_UNKNOWN,
      QMI8658_STATE_RESETTING,
      QMI8658_STATE_RESET,
      QMI8658_STATE_INITIALIZED,
      QMI8658_STATE_ONLINE
    } qmi8658_state_t;

    class QMI8658Component : public PollingComponent, public i2c::I2CDevice
    {
    public:
      void setup() override;
      void dump_config() override;

      void loop() override;
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

      void set_no_motion_detected_sensor(binary_sensor::BinarySensor *no_motion_detected_sensor) { no_motion_detected_sensor_ = no_motion_detected_sensor; };
      void set_any_motion_detected_sensor(binary_sensor::BinarySensor *any_motion_detected_sensor) { any_motion_detected_sensor_ = any_motion_detected_sensor; };
      void set_significant_motion_detected_sensor(binary_sensor::BinarySensor *significant_motion_detected_sensor) { significant_motion_detected_sensor_ = significant_motion_detected_sensor; };

      void set_interrupt1_pin(GPIOPin *interrupt1_pin) {interrupt1_pin_ = interrupt1_pin; };
      void set_interrupt2_pin(GPIOPin *interrupt2_pin) {interrupt2_pin_ = interrupt2_pin; };
      
      protected:
      sensor::Sensor *accel_x_sensor_{nullptr};
      sensor::Sensor *accel_y_sensor_{nullptr};
      sensor::Sensor *accel_z_sensor_{nullptr};
      sensor::Sensor *gyro_x_sensor_{nullptr};
      sensor::Sensor *gyro_y_sensor_{nullptr};
      sensor::Sensor *gyro_z_sensor_{nullptr};

      sensor::Sensor *temperature_sensor_{nullptr};

      binary_sensor::BinarySensor *no_motion_detected_sensor_{nullptr};
      binary_sensor::BinarySensor *any_motion_detected_sensor_{nullptr};
      binary_sensor::BinarySensor *significant_motion_detected_sensor_{nullptr};

      // Control Registers
      i2c::I2CRegister ctrl1_register_ = this->reg(QMI8658_REG_CTRL1);
      i2c::I2CRegister ctrl2_register_ = this->reg(QMI8658_REG_CTRL2);
      i2c::I2CRegister ctrl3_register_ = this->reg(QMI8658_REG_CTRL3);
      i2c::I2CRegister ctrl4_register_ = this->reg(QMI8658_REG_CTRL4);
      i2c::I2CRegister ctrl5_register_ = this->reg(QMI8658_REG_CTRL5);
      i2c::I2CRegister ctrl6_register_ = this->reg(QMI8658_REG_CTRL6);
      i2c::I2CRegister ctrl7_register_ = this->reg(QMI8658_REG_CTRL7);
      i2c::I2CRegister ctrl8_register_ = this->reg(QMI8658_REG_CTRL8);
      i2c::I2CRegister ctrl9_register_ = this->reg(QMI8658_REG_CTRL9);

      // Calibration Registers
      i2c::I2CRegister cal1_l_register_ = this->reg(QMI8658_REG_CAL1_L);
      i2c::I2CRegister cal1_h_register_ = this->reg(QMI8658_REG_CAL1_H);
      i2c::I2CRegister cal2_l_register_ = this->reg(QMI8658_REG_CAL2_L);
      i2c::I2CRegister cal2_h_register_ = this->reg(QMI8658_REG_CAL2_H);
      i2c::I2CRegister cal3_l_register_ = this->reg(QMI8658_REG_CAL3_L);
      i2c::I2CRegister cal3_h_register_ = this->reg(QMI8658_REG_CAL3_H);
      i2c::I2CRegister cal4_l_register_ = this->reg(QMI8658_REG_CAL4_L);
      i2c::I2CRegister cal4_h_register_ = this->reg(QMI8658_REG_CAL4_H);

      i2c::I2CRegister statusint_register_ = this->reg(QMI8658_REG_STATUSINT);
      i2c::I2CRegister status0_register_ = this->reg(QMI8658_REG_STATUS0);
      i2c::I2CRegister status1_register_ = this->reg(QMI8658_REG_STATUS1);

      i2c::I2CRegister reset_register_ = this->reg(QMI8658_REG_RESET);
            
      uint16_t accel_output_data_rate_{};

      GPIOPin *interrupt1_pin_{nullptr};
      void handle_interrupt1_();
      GPIOPin *interrupt2_pin_{nullptr};
      void handle_interrupt2_();

      qmi8658_state_t current_state_ = QMI8658_STATE_UNKNOWN;

      void configure_accel_output_data_rate_();
      float read_accel_gyro_data_(const uint8_t lower_register, const uint8_t higher_register, float sensitivity);
      void initialize_();
      void configure_motion_interrupts_();

    };
    ;

  } // namespace qmi8658
} // namespace esphome
