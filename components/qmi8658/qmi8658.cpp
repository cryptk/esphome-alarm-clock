#include "qmi8658.h" // Include the header we'll create below
#include "esphome/core/log.h"

namespace esphome
{
  namespace qmi8658
  {

    static const char *const TAG = "qmi8658";

    typedef enum
    {
      ACC_SELF_TEST_DISENABLE = 0,
      ACC_SELF_TEST_ENABLE,
    } qmi8658_acc_self_test_t;
    static constexpr uint8_t QMI8658_ACC_SELF_TEST_OFFSET = 7;

    typedef enum
    {
      ACC_SCALE_2G = 0,
      ACC_SCALE_4G,
      ACC_SCALE_8G,
      ACC_SCALE_16G,
    } qmi8658_acc_scale_t;
    static constexpr uint8_t QMI8658_ACC_SCALE_OFFSET = 4;

    typedef enum
    {
      ACC_OUTPUT_DATA_RATE_1000HZ = 3,
      ACC_OUTPUT_DATA_RATE_500HZ,
      ACC_OUTPUT_DATA_RATE_250HZ,
      ACC_OUTPUT_DATA_RATE_125HZ,
      ACC_OUTPUT_DATA_RATE_62_5HZ,
      ACC_OUTPUT_DATA_RATE_31_25HZ,
    } qmi8658_acc_odr_t;
    static constexpr uint8_t QMI8658_ACC_OUTPUT_DATA_RATE_OFFSET = 0;

    typedef enum
    {
      GYRO_SELF_TEST_DISENABLE = 0,
      GYRO_SELF_TEST_ENABLE,
    } qmi8658_gyro_self_test_t;
    static constexpr uint8_t QMI8658_GYRO_SELF_TEST_OFFSET = 7;

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
    static constexpr uint8_t QMI8658_GYRO_SCALE_OFFSET = 4;

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
    static constexpr uint8_t QMI8658_GYRO_OUTPUT_DATA_RATE_OFFSET = 0;

    static constexpr uint8_t QMI8658_STATUS1_ANY_MOTION_MASK = 0x20;
    static constexpr uint8_t QMI8658_STATUS1_NO_MOTION_MASK = 0x40;
    static constexpr uint8_t QMI8658_STATUS1_SIGNIFICANT_MOTION_MASK = 0x80;

    // const float GRAVITY_EARTH = 9.80665f;

    void QMI8658Component::initialize_()
    {
      if (this->current_state_ == QMI8658_STATE_RESETTING) {
        // If we are still resetting, we need to check if we are complete yet
        uint8_t reset_status;
        this->read_register(QMI8658_REG_dQY_L, &reset_status, 1);
        if (reset_status != 0x80) {
          // We have not completed reset yet, try again on next loop
          return;
        }
        // We have successfully reset
        this->current_state_ = QMI8658_STATE_RESET;
      }

      // Continue initialization

      // Enable the internal 2MHz oscillator
      this->ctrl1_register_ &= 0xFE; // ensure bit 1 is 0
      // Enable automatic address incrementation for faster block reads
      this->ctrl1_register_ |= 0x40; // ensure bit 6 is 1

      // Set Acceleromenter Output Data Rate to 1000Hz
      this->configure_accel_output_data_rate_();

      // Set accelerometer to full-scale +/- 2g
      uint8_t SET_ACCEL_SCALE = 0x03;
      this->ctrl2_register_ |= SET_ACCEL_SCALE;

      // Set Gyro Output Data Rate to 896.8Hz, Full-scale +/- 2048dps
      uint8_t SET_GYRO_CONFIG = 0x73;
      this->ctrl3_register_ |= SET_GYRO_CONFIG;

      this->configure_motion_interrupts_();

      // Set the flag to turn on the Accelerometer and Gyroscope
      // Enable Accelerometer (bit 1) and Gyroscope (bit 2)
      uint8_t SET_GYRO_ACCEL_ENABLE = 0x03;
      this->ctrl7_register_ |= SET_GYRO_ACCEL_ENABLE;

      // Enable Any, No and Significant Motion engines (this might need to be done after accelerometer is enabled)
      this->ctrl8_register_ |= 0x0E;

      ESP_LOGD(TAG, "Initialization complete");
      this->current_state_ = QMI8658_STATE_ONLINE;
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

      // Reset the QMI8658 so we start from a known state
      this->reset_register_ |= 0xB0; // table 27 in datasheet
      this->current_state_ = QMI8658_STATE_RESETTING;
      // There is ~15ms of delay for the reset to complete, we will pick initialization back up on the next update() loop
    }

    void QMI8658Component::dump_config()
    {
      ESP_LOGCONFIG(TAG, "QMI8658:");
      LOG_I2C_DEVICE(this);
      if (this->is_failed())
      {
        ESP_LOGE(TAG, "Communication with QMI8658 failed!");
      }
      // LOG_UPDATE_INTERVAL(this);
      LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
      LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
      LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
      LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
      LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
      LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
      LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
      ESP_LOGCONFIG(TAG, "  Acceleration Output Data Rate: %d", this->accel_output_data_rate_);
    }

    void QMI8658Component::update()
    {
      // If we are not yet fully online...
      if (this->current_state_ != QMI8658_STATE_ONLINE) {
        // do not run, we will attempt to initialize in the loop()
        return;
      }

      // if (this->interrupt1_pin_ != nullptr) {
      //   handle_interrupt1_();
      // }

      // // if (this->interrupt2_pin_ != nullptr) {
      // //   handle_interrupt2_();
      // // }

      // Read accelerometer values
      float ax = this->read_accel_gyro_data_(QMI8658_REG_AX_L, QMI8658_REG_AX_H, 16384.0f);
      float ay = this->read_accel_gyro_data_(QMI8658_REG_AY_L, QMI8658_REG_AY_H, 16384.0f);
      float az = this->read_accel_gyro_data_(QMI8658_REG_AZ_L, QMI8658_REG_AZ_H, 16384.0f);

      // Read gyroscope data
      float gx = this->read_accel_gyro_data_(QMI8658_REG_GX_L, QMI8658_REG_GX_H, 16.0f);
      float gy = this->read_accel_gyro_data_(QMI8658_REG_GY_L, QMI8658_REG_GY_H, 16.0f);
      float gz = this->read_accel_gyro_data_(QMI8658_REG_GZ_L, QMI8658_REG_GZ_H, 16.0f);

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

    void QMI8658Component::loop() {
      // If we are not yet fully online...
      if (this->current_state_ != QMI8658_STATE_ONLINE) {
        // ...attempt to initialize
        return this->initialize_();
      }

      if (this->interrupt1_pin_->digital_read()) {
        // Pull the status1 register a single time so we can check the bits below
        auto register_value = this->status1_register_.get();

        if (this->no_motion_detected_sensor_ != nullptr) {
          this->no_motion_detected_sensor_->publish_state(register_value & QMI8658_STATUS1_NO_MOTION_MASK);
        }
        if (this->any_motion_detected_sensor_ != nullptr) {
          this->any_motion_detected_sensor_->publish_state(register_value & QMI8658_STATUS1_ANY_MOTION_MASK);
        }
        if (this->significant_motion_detected_sensor_ != nullptr) {
          this->significant_motion_detected_sensor_->publish_state(register_value & QMI8658_STATUS1_SIGNIFICANT_MOTION_MASK);
        }
      }
    }

    float QMI8658Component::get_setup_priority() const { return setup_priority::DATA; }

    void QMI8658Component::configure_motion_interrupts_() {
      // TODO: This should be togglable and configurable functionality

      // Determines how much motion there must be
      // This is a difference in value between subsequent samples measured in units of 1/32nd of a g of acceleration
      auto AnyMotionXThr = 0xF0;
      auto AnyMotionYThr = 0xF0;
      auto AnyMotionZThr = 0xF0;
      // measured in number of samples based on the Accelerometer Output Data Rate
      // 0x0A == 10
      auto AnyMotionWindow = 0x0A;

      auto NoMotionXThr = 0x80;
      auto NoMotionYThr = 0x80;
      auto NoMotionZThr = 0x80;
      // measured in number of samples based on the Accelerometer Output Data Rate
      // 0xC8 == 200
      auto NoMotionWindow = 0xC8;

      auto CTRL_CMD_CONFIGURE_MOTION = 0x0E;
      // ESP_LOGD(TAG, "Register status: %d", this->cal1_l_register_.get());
      // range 0-255, 0x80 == 128
      // AnyMotionXThr
      this->cal1_l_register_ = AnyMotionXThr;
      // AnyMotionYThr
      this->cal1_h_register_ = AnyMotionYThr;
      // AnyMotionZThr
      this->cal2_l_register_ = AnyMotionZThr;

      // range 0-255, 0x40 == 64
      // AnyMotionXThr
      this->cal2_h_register_ = NoMotionXThr;
      // AnyMotionYThr
      this->cal3_l_register_ = NoMotionYThr;
      // AnyMotionZThr
      this->cal3_h_register_ = NoMotionZThr;

      //MOTION_MODE_CTRL
      // 0xF7 == NoMotion requires ALL axises to be still (AND), AnyMotion requires ANY axis to have motion (OR)
      // 0xA2 == Enable No Motion and Any Motion on X-Axis only
      this->cal4_l_register_ = 0xA2;

      // Sending first configuration command
      this->cal4_h_register_ = 0x01;

      // Push config
      this->ctrl9_register_ = CTRL_CMD_CONFIGURE_MOTION;

      // AnyMotionWindow
      // TODO: Validate that these registers are zeroed out after the CTRL_CMD_CONFIGURE_MOTION command above before writing to them again
      // ESP_LOGD(TAG, "Register status: %d", this->cal1_l_register_.get());
      this->cal1_l_register_ = AnyMotionWindow;

      //NoMotionWindow
      this->cal1_h_register_ = NoMotionWindow;

      // SigMotionWaitWindow == 1000 samples
      this->cal2_l_register_ = 0xE8;
      this->cal2_h_register_ = 0x03;

      // SigMotionConfirmWindow == 1500 samples
      this->cal3_l_register_ = 0xDC;
      this->cal3_h_register_ = 0x05;

      // Sending second configuration command
      this->cal4_h_register_ = 0x02;

      // Push config
      this->ctrl9_register_ = CTRL_CMD_CONFIGURE_MOTION;



      // Set motion engine to use interrupt 1 pin
      this->ctrl8_register_ |= 0x40;

      // Enable Int1 pin
      this->ctrl1_register_ = 0x08;
      
    }

    // void QMI8658Component::handle_interrupt1_() {
    //   // ESP_LOGD(TAG, "Handling Interrupt1 Pin");
    //   bool int1_set = this->interrupt1_pin_->digital_read();
    //   // ESP_LOGD(TAG, "Status of Int1: %d", int1_set);
    //   auto register_value = this->status1_register_.get();
    //   // ESP_LOGD(TAG, "Status of the status1 register: %X", register_value);
    //   auto any_motion_detected = register_value & QMI8658_STATUS1_ANY_MOTION_MASK;
    //   auto no_motion_detected = register_value & QMI8658_STATUS1_NO_MOTION_MASK;
    //   auto sig_motion_detected = register_value & QMI8658_STATUS1_SIG_MOTION_MASK;
    //   // ESP_LOGD(TAG, "Any Motion: %d; No Motion: %d; Sig Motion: %d", any_motion_detected>1, no_motion_detected>1, sig_motion_detected>1);
    // }

    // void QMI8658Component::handle_interrupt2_() {
    //   // ESP_LOGD(TAG, "Handling Interrupt2 Pin");
    //   bool int2_set = this->interrupt2_pin_->digital_read();
    //   // ESP_LOGD(TAG, "Status of Int2: %d", int2_set);
    // }

    float QMI8658Component::read_accel_gyro_data_(const uint8_t lower_register, const uint8_t higher_register, float sensitivity) {
      uint8_t data[2];
      this->read_register(lower_register, &data[0], 1);
      this->read_register(higher_register, &data[1], 1);
      int16_t raw_value = (int16_t)((data[1] << 8) | data[0]);
      return raw_value / sensitivity;
    }

    void QMI8658Component::configure_accel_output_data_rate_()
    {
      uint8_t data_rate_value;
      switch (this->accel_output_data_rate_)
      {
      case 1000:
        data_rate_value = ACC_OUTPUT_DATA_RATE_1000HZ;
        break;
      case 500:
        data_rate_value = ACC_OUTPUT_DATA_RATE_500HZ;
        break;
      case 250:
        data_rate_value = ACC_OUTPUT_DATA_RATE_250HZ;
        break;
      case 125:
        data_rate_value = ACC_OUTPUT_DATA_RATE_125HZ;
        break;
      case 63:
        data_rate_value = ACC_OUTPUT_DATA_RATE_62_5HZ;
        break;
      case 31:
        data_rate_value = ACC_OUTPUT_DATA_RATE_31_25HZ;
        break;
      }

      // TODO: We should just return the value back to the caller and let them get a register with this->reg() and update it with a |= operation
      data_rate_value << QMI8658_ACC_OUTPUT_DATA_RATE_OFFSET;
      this->ctrl1_register_ |= data_rate_value;
      // this->write_register(QMI8658_REG_CTRL1, &data_rate_value, 1);
    }

  } // namespace qmi8658
} // namespace esphome
