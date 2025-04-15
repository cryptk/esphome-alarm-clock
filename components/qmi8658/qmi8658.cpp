#include "qmi8658.h" // Include the header we'll create below
#include "esphome/core/log.h"

namespace esphome
{
  namespace qmi8658
  {

    static const char *const TAG = "qmi8658";

    typedef enum
    {
      ACCEL_SELF_TEST_DISENABLE = 0,
      ACCEL_SELF_TEST_ENABLE,
    } qmi8658_accel_self_test_t;
    static constexpr uint8_t QMI8658_ACCEL_SELF_TEST_OFFSET = 7;

    typedef enum
    {
      ACCEL_SCALE_2G = 0x00,
      ACCEL_SCALE_4G = 0x10,
      ACCEL_SCALE_8G = 0x20,
      ACCEL_SCALE_16G = 0x30,
    } qmi8658_accel_scale_t;
    static constexpr uint8_t QMI8658_ACCEL_SCALE_MASK = 0x70;

    typedef enum
    {
      ACCEL_OUTPUT_DATA_RATE_1000HZ = 0x03,
      ACCEL_OUTPUT_DATA_RATE_500HZ = 0x04,
      ACCEL_OUTPUT_DATA_RATE_250HZ = 0x05,
      ACCEL_OUTPUT_DATA_RATE_125HZ = 0x06,
      ACCEL_OUTPUT_DATA_RATE_62_5HZ = 0x07,
      ACCEL_OUTPUT_DATA_RATE_31_25HZ = 0x08,
      // TODO: There are some low-power data rates for the accelerometer
      // those should be implemented after we make sure there isn't any other
      // special config needed for low-power operation
    } qmi8658_accel_odr_t;
    static constexpr uint8_t QMI8658_ACCEL_OUTPUT_DATA_RATE_MASK = 0x0F;

    typedef enum
    {
      GYRO_SELF_TEST_DISENABLE = 0,
      GYRO_SELF_TEST_ENABLE,
    } qmi8658_gyro_self_test_t;
    static constexpr uint8_t QMI8658_GYRO_SELF_TEST_OFFSET = 7;

    typedef enum
    {
      GYRO_SCALE_16DPS = 0x00,
      GYRO_SCALE_32DPS = 0x10,
      GYRO_SCALE_64DPS = 0x20,
      GYRO_SCALE_128DPS = 0x30,
      GYRO_SCALE_256DPS = 0x40,
      GYRO_SCALE_512DPS = 0x50,
      GYRO_SCALE_1024DPS = 0x60,
      GYRO_SCALE_2048DPS = 0x70,
    } qmi8658_gyro_scale_t;
    static constexpr uint8_t QMI8658_GYRO_SCALE_MASK = 0x70;

    typedef enum
    {
      GYRO_OUTPUT_DATA_RATE_7174_4HZ = 0x00,
      GYRO_OUTPUT_DATA_RATE_3584_2HZ = 0x01,
      GYRO_OUTPUT_DATA_RATE_1793_6HZ = 0x02,
      GYRO_OUTPUT_DATA_RATE_896_8HZ = 0x03,
      GYRO_OUTPUT_DATA_RATE_448_4HZ = 0x04,
      GYRO_OUTPUT_DATA_RATE_224_2HZ = 0x05,
      GYRO_OUTPUT_DATA_RATE_112_1HZ = 0x06,
      GYRO_OUTPUT_DATA_RATE_56_05HZ = 0x07,
      GYRO_OUTPUT_DATA_RATE_28_025HZ = 0x08,
    } qmi8658_gyro_odr_t;
    static constexpr uint8_t QMI8658_GYRO_OUTPUT_DATA_RATE_MASK = 0x0F;

    static constexpr uint8_t QMI8658_STATUS1_ANY_MOTION_MASK = 0x20;
    static constexpr uint8_t QMI8658_STATUS1_NO_MOTION_MASK = 0x40;
    static constexpr uint8_t QMI8658_STATUS1_SIGNIFICANT_MOTION_MASK = 0x80;

    // const float GRAVITY_EARTH = 9.80665f;

    void QMI8658Component::initialize_()
    {
      ESP_LOGD(TAG, "Attempting to initialize");
      if (this->current_state_ == QMI8658_STATE_RESETTING) {
        // If we are still resetting, we need to check if we are complete yet
        // TODO use the reg() objects in the component here
        uint8_t reset_status;
        this->read_register(QMI8658_REG_dQY_L, &reset_status, 1);
        if (reset_status != 0x80) {
          // We have not completed reset yet, try again on next loop
          return;
        }
        // We have successfully reset
        this->current_state_ = QMI8658_STATE_RESET;
      }
      ESP_LOGD(TAG, "Reset complete, continuing with initialization");

      // Continue initialization

      // Enable the internal 2MHz oscillator
      this->ctrl1_register_ &= 0xFE; // ensure bit 0 is 0
      // Enable automatic address incrementation for burst reads
      // These aren't implemented in the component yet
      this->ctrl1_register_ |= 0x40; // ensure bit 6 is 1

      this->configure_accel_output_data_rate_();
      this->configure_accel_scale_();

      this->configure_gyro_output_data_rate_();
      this->configure_gyro_scale_();

      // Disable DataReady Interrupt
      this->ctrl7_register_ |= 0x20;

      // Set the flag to turn on the Accelerometer and Gyroscope
      // Enable Accelerometer (bit 1) and Gyroscope (bit 2)
      uint8_t SET_GYRO_ACCEL_ENABLE = 0x03;
      this->ctrl7_register_ |= SET_GYRO_ACCEL_ENABLE;

      ESP_LOGD(TAG, "Initialization complete");
      this->current_state_ = QMI8658_STATE_ONLINE;
    }

    void QMI8658Component::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up QMI8658...");

      // Ask the chip for an identifier.  0x05 means it's a QST sensor.
      // TODO use the reg() objects in the component here
      uint8_t who_am_i;
      if (!this->read_byte(QMI8658_REG_WHO_AM_I, &who_am_i) || (who_am_i != 0x05))
      {
        ESP_LOGE(TAG, "WHOAMI check failed");
        this->mark_failed();
        return;
      }
      uint8_t revision_id;
      if (!this->read_byte(QMI8658_REG_REVISION_ID, &revision_id))
      {
        ESP_LOGE(TAG, "Revision ID check failed");
        this->mark_failed();
        return;
      }
      ESP_LOGCONFIG(TAG, "Device revision: %d", revision_id);

      // Reset the QMI8658 so we start from a known state
      ESP_LOGD(TAG, "Resetting qmi8658");
      this->reset_register_ |= 0xB0; // table 27 in datasheet
      this->current_state_ = QMI8658_STATE_RESETTING;
      ESP_LOGD(TAG, "Setup complete, will continue initialization after reset is complete");

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
      ESP_LOGCONFIG(TAG, "  Accelerometer Output Data Rate: %d", this->accel_output_data_rate_);
      ESP_LOGCONFIG(TAG, "  Accelerometer Scale: %d", this->accel_scale_);
      ESP_LOGCONFIG(TAG, "  Gyroscope Output Data Rate: %d", this->gyro_output_data_rate_);
      ESP_LOGCONFIG(TAG, "  Gyroscope Output Data Rate: %d", this->gyro_scale_);
    }

    float QMI8658Component::get_setup_priority() const { return setup_priority::DATA; }

    void QMI8658Component::configure_accel_output_data_rate_()
    {
      // Ensure the accelerometer Output Data Rate bits are all set to 0
      this->ctrl2_register_ &= ~QMI8658_ACCEL_OUTPUT_DATA_RATE_MASK;
      // Set the accelerometer Output Data Rate bits depending on our desired value
      if (this->accel_output_data_rate_ == 1000) {this->ctrl2_register_ |= ACCEL_OUTPUT_DATA_RATE_1000HZ; return;};
      if (this->accel_output_data_rate_ == 500) {this->ctrl2_register_ |= ACCEL_OUTPUT_DATA_RATE_500HZ; return;};
      if (this->accel_output_data_rate_ == 250) {this->ctrl2_register_ |= ACCEL_OUTPUT_DATA_RATE_250HZ; return;};
      if (this->accel_output_data_rate_ == 125) {this->ctrl2_register_ |= ACCEL_OUTPUT_DATA_RATE_125HZ; return;};
      if (this->accel_output_data_rate_ == 63) {this->ctrl2_register_ |= ACCEL_OUTPUT_DATA_RATE_62_5HZ; return;};
      if (this->accel_output_data_rate_ == 31) {this->ctrl2_register_ |= ACCEL_OUTPUT_DATA_RATE_31_25HZ; return;};
      ESP_LOGE(TAG, "Failed to configure accelerometer output data rate to %f", this->accel_output_data_rate_);
      this->mark_failed();
    }

    void QMI8658Component::configure_accel_scale_() 
    {
      // Ensure the accelerometer scale bits are all set to 0
      this->ctrl2_register_ &= ~QMI8658_ACCEL_SCALE_MASK;
      // Set the accelerometer scale bits depending on our desired value
      if (this->accel_scale_ == 2) {this->ctrl2_register_ |= ACCEL_SCALE_2G; return;};
      if (this->accel_scale_ == 4) {this->ctrl2_register_ |= ACCEL_SCALE_4G; return;};
      if (this->accel_scale_ == 8) {this->ctrl2_register_ |= ACCEL_SCALE_8G; return;};
      if (this->accel_scale_ == 16) {this->ctrl2_register_ |= ACCEL_SCALE_16G; return;};
      ESP_LOGE(TAG, "Failed to configure accelerometer scale to %d", this->accel_scale_);
      this->mark_failed();
    }

    void QMI8658Component::configure_gyro_output_data_rate_()
    {
      // Ensure the gyro Output Data Rate bits are all set to 0
      this->ctrl2_register_ &= ~QMI8658_GYRO_OUTPUT_DATA_RATE_MASK;
      // Set the gyro Output Data Rate bits depending on our desired value
      if (this->gyro_output_data_rate_ == 7174) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_7174_4HZ; return;};
      if (this->gyro_output_data_rate_ == 3587) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_3584_2HZ; return;};
      if (this->gyro_output_data_rate_ == 1794) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_1793_6HZ; return;};
      if (this->gyro_output_data_rate_ == 897) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_896_8HZ; return;};
      if (this->gyro_output_data_rate_ == 448) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_448_4HZ; return;};
      if (this->gyro_output_data_rate_ == 224) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_224_2HZ; return;};
      if (this->gyro_output_data_rate_ == 112) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_112_1HZ; return;};
      if (this->gyro_output_data_rate_ == 56) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_56_05HZ; return;};
      if (this->gyro_output_data_rate_ == 28) {this->ctrl3_register_ |= GYRO_OUTPUT_DATA_RATE_28_025HZ; return;};
      ESP_LOGE(TAG, "Failed to configure gyro output data rate to %f", this->gyro_output_data_rate_);
      this->mark_failed();
    }

    void QMI8658Component::configure_gyro_scale_()
    {
      // Ensure the gyro scale bits are all set to 0
      this->ctrl3_register_ &= ~QMI8658_GYRO_SCALE_MASK;
      // Set the gyro scale bits depending on our desired value
      if (this->gyro_scale_ == 16) {this->ctrl3_register_ |= GYRO_SCALE_16DPS; return;};
      if (this->gyro_scale_ == 32) {this->ctrl3_register_ |= GYRO_SCALE_32DPS; return;};
      if (this->gyro_scale_ == 64) {this->ctrl3_register_ |= GYRO_SCALE_64DPS; return;};
      if (this->gyro_scale_ == 128) {this->ctrl3_register_ |= GYRO_SCALE_128DPS; return;};
      if (this->gyro_scale_ == 256) {this->ctrl3_register_ |= GYRO_SCALE_256DPS; return;};
      if (this->gyro_scale_ == 512) {this->ctrl3_register_ |= GYRO_SCALE_512DPS; return;};
      if (this->gyro_scale_ == 1024) {this->ctrl3_register_ |= GYRO_SCALE_1024DPS; return;};
      if (this->gyro_scale_ == 2048) {this->ctrl3_register_ |= GYRO_SCALE_2048DPS; return;};
      ESP_LOGE(TAG, "Failed to configure gyro scale to %d", this->gyro_scale_);
      this->mark_failed();
    }

    void QMI8658Component::loop() {
    //   // If we are not yet fully online...
      // ESP_LOGD(TAG, "Loop Running");
      if (this->current_state_ != QMI8658_STATE_ONLINE) {
        // ...attempt to initialize
        ESP_LOGD(TAG, "Current State is: %d", this->current_state_);
        return this->initialize_();
      }
      return;
    }

  } // namespace qmi8658
} // namespace esphome
