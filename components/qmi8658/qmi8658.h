#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace qmi8658 {

// Datasheet: https://www.qstcorp.com/upload/pdf/202301/13-52-25%20QMI8658A%20Datasheet%20Rev%20A.pdf

// General Purpose Registers
static constexpr uint8_t QMI8658_REG_WHO_AM_I = 0x00;     // Device Identifier
static constexpr uint8_t QMI8658_REG_REVISION_ID = 0x01;  // Device Revision ID

// Setup and Control Registers
static constexpr uint8_t QMI8658_REG_CTRL1 = 0x02;  // SPI Interface and Sensor Enable
typedef enum {
  QMI8658_CTRL1_SPI_WIRE_COUNT_MASK = 0x80,
  QMI8658_CTRL1_ADDRESS_AUTO_INCREMENT_MASK = 0x40,
  QMI8658_CTRL1_USE_BIG_ENDIAN_MASK = 0x20,
  QMI8658_CTRL1_INT2_EN_MASK = 0x10,
  QMI8658_CTRL1_INT1_EN_MASK = 0x08,
  QMI8658_CTRL1_FIFO_INT_SEL_MASK = 0x04,
  QMI8658_CTRL1_INTERNAL_OSCILLATOR_ENABLE_MASK = 0x01,
} qmi8658_ctrl1_mask_t;

static constexpr uint8_t QMI8658_REG_CTRL2 = 0x03;  // Accelerometer: Output Data Rate, Full Scale, Self Test
typedef enum {
  QMI8658_CTRL2_ACCEL_SELF_TEST_MASK = 0x80,
  QMI8658_CTRL2_ACCEL_SCALE_MASK = 0x70,
  QMI8658_CTRL2_ACCEL_OUTPUT_DATA_RATE_MASK = 0x0F,
} qmi8658_ctrl2_mask_t;
typedef enum {
  QMI8658_ACCEL_SCALE_2G = 0x00,
  QMI8658_ACCEL_SCALE_4G = 0x10,
  QMI8658_ACCEL_SCALE_8G = 0x20,
  QMI8658_ACCEL_SCALE_16G = 0x30,
} qmi8658_accel_scale_t;
typedef enum {
  QMI8658_ACCEL_OUTPUT_DATA_RATE_1000HZ = 0x03,
  QMI8658_ACCEL_OUTPUT_DATA_RATE_500HZ = 0x04,
  QMI8658_ACCEL_OUTPUT_DATA_RATE_250HZ = 0x05,
  QMI8658_ACCEL_OUTPUT_DATA_RATE_125HZ = 0x06,
  QMI8658_ACCEL_OUTPUT_DATA_RATE_62_5HZ = 0x07,
  QMI8658_ACCEL_OUTPUT_DATA_RATE_31_25HZ = 0x08,
  // TODO: There are some low-power data rates for the accelerometer
  // those should be implemented after we make sure there isn't any other
  // special config needed for low-power operation
} qmi8658_accel_odr_t;

static constexpr uint8_t QMI8658_REG_CTRL3 = 0x04;  // Gyroscope: Output Data Rate, Full Scale, Self Test
typedef enum {
  QMI8658_CTRL3_GYRO_SELF_TEST_MASK = 0x80,
  QMI8658_CTRL3_GYRO_SCALE_MASK = 0x70,
  QMI8658_CTRL3_GYRO_OUTPUT_DATA_RATE_MASK = 0x0F,
} qmi8658_ctrl3_mask_t;
typedef enum {
  QMI8658_GYRO_SCALE_16DPS = 0x00,
  QMI8658_GYRO_SCALE_32DPS = 0x10,
  QMI8658_GYRO_SCALE_64DPS = 0x20,
  QMI8658_GYRO_SCALE_128DPS = 0x30,
  QMI8658_GYRO_SCALE_256DPS = 0x40,
  QMI8658_GYRO_SCALE_512DPS = 0x50,
  QMI8658_GYRO_SCALE_1024DPS = 0x60,
  QMI8658_GYRO_SCALE_2048DPS = 0x70,
} qmi8658_gyro_scale_t;
typedef enum {
  QMI8658_GYRO_OUTPUT_DATA_RATE_7174_4HZ = 0x00,
  QMI8658_GYRO_OUTPUT_DATA_RATE_3584_2HZ = 0x01,
  QMI8658_GYRO_OUTPUT_DATA_RATE_1793_6HZ = 0x02,
  QMI8658_GYRO_OUTPUT_DATA_RATE_896_8HZ = 0x03,
  QMI8658_GYRO_OUTPUT_DATA_RATE_448_4HZ = 0x04,
  QMI8658_GYRO_OUTPUT_DATA_RATE_224_2HZ = 0x05,
  QMI8658_GYRO_OUTPUT_DATA_RATE_112_1HZ = 0x06,
  QMI8658_GYRO_OUTPUT_DATA_RATE_56_05HZ = 0x07,
  QMI8658_GYRO_OUTPUT_DATA_RATE_28_025HZ = 0x08,
} qmi8658_gyro_odr_t;

static constexpr uint8_t QMI8658_REG_CTRL4 = 0x05;  // Reserved

static constexpr uint8_t QMI8658_REG_CTRL5 = 0x06;  // Low pass filter setting.
typedef enum {
  QMI8658_CTRL5_GYRO_LOW_PASS_FILTER_MODE_MASK = 0x60,
  QMI8658_CTRL5_GYRO_LOW_PASS_FILTER_ENABLE_MASK = 0x10,
  QMI8658_CTRL5_ACCEL_LOW_PASS_FILTER_MODE_MASK = 0x06,
  QMI8658_CTRL5_ACCEL_LOW_PASS_FILTER_ENABLE_MASK = 0x01,
} qmi8658_ctrl5_mask_t;

static constexpr uint8_t QMI8658_REG_CTRL6 = 0x07;  // Reserved

static constexpr uint8_t QMI8658_REG_CTRL7 = 0x08;  // Enable Sensors
typedef enum {
  QMI8658_CTRL7_SYNC_SAMPLE_MASK = 0x80,
  QMI8658_CTRL7_DATA_READY_DISABLE_MASK = 0x20,
  QMI8658_CTRL7_GYRO_SNOOZE_MASK = 0x10,
  QMI8658_CTRL7_GYRO_ENABLE_MASK = 0x02,
  QMI8658_CTRL7_ACCEL_ENABLE_MASK = 0x01,
} qmi8658_ctrl7_mask_t;

static constexpr uint8_t QMI8658_REG_CTRL8 = 0x09;  // Motion Detection Control
typedef enum {
  QMI8658_CTRL8_SET_CTRL9_HANDSHAKE_TYPE_MASK = 0x80,
  QMI8658_CTRL8_ACTIVITY_INT_SELECT_MASK = 0x40,
  QMI8658_CTRL8_PEDOMETER_ENABLE_MASK = 0x10,
  QMI8658_CTRL8_SIGNIFICANT_MOTION_ENABLE_MASK = 0x08,
  QMI8658_CTRL8_NO_MOTION_ENABLE_MASK = 0x04,
  QMI8658_CTRL8_ANY_MOTION_ENABLE_MASK = 0x02,
  QMI8658_CTRL8_TAP_ENABLE_MASK = 0x01,
} qmi8658_ctrl8_mask_t;

static constexpr uint8_t QMI8658_REG_CTRL9 = 0x0A;  // Host Commands
typedef enum {
  QMI8658_CTRL9_CMD_ACK = 0x00,
  QMI8658_CTRL9_CMD_RST_FIFO = 0x04,
  QMI8658_CTRL9_CMD_REQ_FIFO = 0x05,
  QMI8658_CTRL9_CMD_WRITE_WOM_SETTING = 0x08,
  QMI8658_CTRL9_CMD_ACCEL_HOST_DELTA_OFFSET = 0x09,
  QMI8658_CTRL9_CMD_GYRO_HOST_DELTA_OFFSET = 0x0A,
  QMI8658_CTRL9_CMD_CONFIGURE_TAP = 0x0C,
  QMI8658_CTRL9_CMD_CONFIGURE_PEDOMETER = 0x0D,
  QMI8658_CTRL9_CMD_CONFIGURE_MOTION = 0x0E,
  QMI8658_CTRL9_CMD_RESET_PEDOMETER = 0x0F,
  QMI8658_CTRL9_CMD_COPY_USID = 0x10,
  QMI8658_CTRL9_CMD_SET_RPU = 0x11,
  QMI8658_CTRL9_CMD_AHB_CLOCK_GATING = 0x12,
  QMI8658_CTRL9_CMD_ON_DEMAND_CALIBRATION = 0xA2,
  QMI8658_CTRL9_CMD_APPLY_GYRO_GAINS = 0xAA,
} qmi8658_ctrl9_cmd_t;
static constexpr uint8_t QMI8658_CTRL9_CMD_DONE = 0x80;

// Host Controlled Calibration Registers (See CTRL9 in datasheet, Usage is Optional)
static constexpr uint8_t QMI8658_REG_CAL1_L = 0x0B;  // Calibration Register 1 lower 8 bits
static constexpr uint8_t QMI8658_REG_CAL1_H = 0x0C;  // Calibration Register 1 upper 8 bits
static constexpr uint8_t QMI8658_REG_CAL2_L = 0x0D;  // Calibration Register 2 lower 8 bits
static constexpr uint8_t QMI8658_REG_CAL2_H = 0x0E;  // Calibration Register 2 upper 8 bits
static constexpr uint8_t QMI8658_REG_CAL3_L = 0x0F;  // Calibration Register 3 lower 8 bits
static constexpr uint8_t QMI8658_REG_CAL3_H = 0x10;  // Calibration Register 3 upper 8 bits
static constexpr uint8_t QMI8658_REG_CAL4_L = 0x11;  // Calibration Register 4 lower 8 bits
static constexpr uint8_t QMI8658_REG_CAL4_H = 0x12;  // Calibration Register 4 upper 8 bits

// FIFO Registers
static constexpr uint8_t QMI8658_REG_FIFO_WTM_TH = 0x13;    // FIFO watermark level, in ODRs
static constexpr uint8_t QMI8658_REG_FIFO_CTRL = 0x14;      // FIFO Setup
static constexpr uint8_t QMI8658_REG_FIFO_SMPL_CNT = 0x15;  // FIFO sample count LSBs
static constexpr uint8_t QMI8658_REG_FIFO_STATUS = 0x16;    // FIFO Status
static constexpr uint8_t QMI8658_REG_FIFO_DATA = 0x17;      // FIFO Data

// Status Registers
static constexpr uint8_t QMI8658_REG_STATUSINT = 0x2D;  // Sensor Data Locking/Available, CmdDone (CTRL9 protocol bit).
static constexpr uint8_t QMI8658_REG_STATUS0 = 0x2E;    // Output Data Over Run and Data Availability.
static constexpr uint8_t QMI8658_REG_STATUS1 = 0x2F;    // Misc Status: Any/No/Significant Motion, Pedometer, Tap.

// Timestamp Register
static constexpr uint8_t QMI8658_REG_TIMESTAMP_LOW = 0x30;   // Timestamp Lower 8 bits
static constexpr uint8_t QMI8658_REG_TIMESTAMP_MID = 0x31;   // Timestamp middle 8 bits
static constexpr uint8_t QMI8658_REG_TIMESTAMP_HIGH = 0x32;  // Timestamp upper 8 bits;

// Data Output Registers (16 bits 2’s Complement Except COD Sensor Data)
static constexpr uint8_t QMI8658_REG_TEMP_L = 0x33;  // Temperature Output Data lower 8 bits
static constexpr uint8_t QMI8658_REG_TEMP_H = 0x34;  // Temperature Output Data upper 8 bits

static constexpr uint8_t QMI8658_REG_AX_L = 0x35;  // X-axis Acceleration lower 8 bits
static constexpr uint8_t QMI8658_REG_AX_H = 0x36;  // X-axis Acceleration upper 8 bits
static constexpr uint8_t QMI8658_REG_AY_L = 0x37;  // Y-axis Acceleration lower 8 bits
static constexpr uint8_t QMI8658_REG_AY_H = 0x38;  // Y-axis Acceleration upper 8 bits
static constexpr uint8_t QMI8658_REG_AZ_L = 0x39;  // Z-axis Acceleration lower 8 bits
static constexpr uint8_t QMI8658_REG_AZ_H = 0x3A;  // Z-axis Acceleration upper 8 bits

static constexpr uint8_t QMI8658_REG_GX_L = 0x3B;  // X-axis Angular Rate lower 8 bits
static constexpr uint8_t QMI8658_REG_GX_H = 0x3C;  // X-axis Angular Rate upper 8 bits
static constexpr uint8_t QMI8658_REG_GY_L = 0x3D;  // Y-axis Angular Rate lower 8 bits
static constexpr uint8_t QMI8658_REG_GY_H = 0x3E;  // Y-axis Angular Rate upper 8 bits
static constexpr uint8_t QMI8658_REG_GZ_L = 0x3F;  // Z-axis Angular Rate lower 8 bits
static constexpr uint8_t QMI8658_REG_GZ_H = 0x40;  // Z-axis Angular Rate upper 8 bits

// COD Indication and General Purpose Registers
static constexpr uint8_t QMI8658_REG_COD_STATUS = 0x46;  // Calibration-On-Demand status register

static constexpr uint8_t QMI8658_REG_dQW_L = 0x49;  // General purpose register
static constexpr uint8_t QMI8658_REG_dQW_H = 0x4A;  // General purpose register
static constexpr uint8_t QMI8658_REG_dQX_L = 0x4B;  // General purpose register
static constexpr uint8_t QMI8658_REG_dQX_H = 0x4C;  // Reserved
static constexpr uint8_t QMI8658_REG_dQY_L = 0x4D;  // General purpose register, contains reset status
static constexpr uint8_t QMI8658_REG_dQY_H = 0x4E;  // Reserved
static constexpr uint8_t QMI8658_REG_dQZ_L = 0x4F;  // Reserved
static constexpr uint8_t QMI8658_REG_dQZ_H = 0x50;  // Reserved

static constexpr uint8_t QMI8658_REG_dVX_L = 0x51;  // General purpose register
static constexpr uint8_t QMI8658_REG_dVX_H = 0x52;  // General purpose register
static constexpr uint8_t QMI8658_REG_dVY_L = 0x53;  // General purpose register
static constexpr uint8_t QMI8658_REG_dVY_H = 0x54;  // General purpose register
static constexpr uint8_t QMI8658_REG_dVZ_L = 0x55;  // General purpose register
static constexpr uint8_t QMI8658_REG_dVZ_H = 0x56;  // General purpose register

static constexpr uint8_t QMI8658_REG_TAP_STATUS = 0x59;     // Axis, direction, number of detected Tap
static constexpr uint8_t QMI8658_REG_STEP_CNT_LOW = 0x5A;   // Low byte of step count of Pedometer
static constexpr uint8_t QMI8658_REG_STEP_CNT_MIDL = 0x5B;  // Middle byte of step count of Pedometer
static constexpr uint8_t QMI8658_REG_STEP_CNT_HIGH = 0x5C;  // High byte of step count of Pedometer

// Reset Register
static constexpr uint8_t QMI8658_REG_RESET = 0x60;  // Soft Reset Register

typedef enum {
  QMI8658_STATE_UNKNOWN,
  QMI8658_STATE_RESETTING,
  QMI8658_STATE_RESET,
  QMI8658_STATE_INITIALIZED,
  QMI8658_STATE_ONLINE
} qmi8658_state_t;

static constexpr uint8_t QMI8658_SOFT_RESET_COMMAND = 0xB0;
static constexpr uint8_t QMI8658_SOFT_RESET_SUCCESS = 0x80;

class QMI8658SensorComponent;
class QMI8658BinarySensorComponent;

class QMI8658Component : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;

  void loop() override;

  float get_setup_priority() const override;

  void set_accel_output_data_rate(uint16_t accel_output_data_rate) {
    accel_output_data_rate_ = accel_output_data_rate;
  };
  void set_accel_scale(uint8_t accel_scale) { accel_scale_ = accel_scale; };
  void set_accel_sensitivity(uint16_t accel_sensitivity) { accel_sensitivity_ = accel_sensitivity; };

  void set_gyro_output_data_rate(uint16_t gyro_output_data_rate) { gyro_output_data_rate_ = gyro_output_data_rate; };
  void set_gyro_scale(uint16_t gyro_scale) { gyro_scale_ = gyro_scale; };
  void set_gyro_sensitivity(uint16_t gyro_sensitivity) { gyro_sensitivity_ = gyro_sensitivity; };

  friend class QMI8658SensorComponent;
  friend class QMI8658BinarySensorComponent;

 protected:
  void initialize_();

  uint16_t accel_output_data_rate_{};
  uint8_t accel_scale_{};
  uint16_t accel_sensitivity_{};
  void configure_accel_output_data_rate_();
  void configure_accel_scale_();

  uint16_t gyro_output_data_rate_{};
  uint16_t gyro_scale_{};
  uint16_t gyro_sensitivity_{};
  void configure_gyro_output_data_rate_();
  void configure_gyro_scale_();

  bool send_ctrl9_command_(qmi8658_ctrl9_cmd_t command);

  qmi8658_state_t current_state_ = QMI8658_STATE_UNKNOWN;

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

  // Accelerometer Registers
  i2c::I2CRegister accel_x_l_register_ = this->reg(QMI8658_REG_AX_L);
  i2c::I2CRegister accel_x_h_register_ = this->reg(QMI8658_REG_AX_H);
  i2c::I2CRegister accel_y_l_register_ = this->reg(QMI8658_REG_AY_L);
  i2c::I2CRegister accel_y_h_register_ = this->reg(QMI8658_REG_AY_H);
  i2c::I2CRegister accel_z_l_register_ = this->reg(QMI8658_REG_AZ_L);
  i2c::I2CRegister accel_z_h_register_ = this->reg(QMI8658_REG_AZ_H);

  // Gyroscope Registers
  i2c::I2CRegister gyro_x_l_register_ = this->reg(QMI8658_REG_GX_L);
  i2c::I2CRegister gyro_x_h_register_ = this->reg(QMI8658_REG_GX_H);
  i2c::I2CRegister gyro_y_l_register_ = this->reg(QMI8658_REG_GY_L);
  i2c::I2CRegister gyro_y_h_register_ = this->reg(QMI8658_REG_GY_H);
  i2c::I2CRegister gyro_z_l_register_ = this->reg(QMI8658_REG_GZ_L);
  i2c::I2CRegister gyro_z_h_register_ = this->reg(QMI8658_REG_GZ_H);

  i2c::I2CRegister statusint_register_ = this->reg(QMI8658_REG_STATUSINT);
  i2c::I2CRegister status0_register_ = this->reg(QMI8658_REG_STATUS0);
  i2c::I2CRegister status1_register_ = this->reg(QMI8658_REG_STATUS1);

  i2c::I2CRegister reset_register_ = this->reg(QMI8658_REG_RESET);
  i2c::I2CRegister dQY_l_register = this->reg(QMI8658_REG_dQY_L);
};

}  // namespace qmi8658
}  // namespace esphome
