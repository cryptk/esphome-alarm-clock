#include "qmi8658_binary_sensor.h" // Include the header we'll create below
#include "esphome/core/log.h"

namespace esphome
{
  namespace qmi8658
  {

    static const char *const TAG = "qmi8658_binary_sensor";

    static constexpr uint8_t QMI8658_STATUS1_ANY_MOTION_MASK = 0x20;
    static constexpr uint8_t QMI8658_STATUS1_NO_MOTION_MASK = 0x40;
    static constexpr uint8_t QMI8658_STATUS1_SIGNIFICANT_MOTION_MASK = 0x80;

    void splitUint16(uint16_t input, uint8_t& lowerByte, uint8_t& higherByte) {
      // Use bitwise AND with a mask to isolate the lower 8 bits.
      lowerByte = static_cast<uint8_t>(input & 0xFF);
    
      // Use right bit shift to move the higher 8 bits to the lower position.
      // Then, cast to uint8_t to store only those bits.
      higherByte = static_cast<uint8_t>((input >> 8) & 0xFF);
    }

    void QMI8658BinarySensorComponent::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up QMI8658 Binary Sensors...");
    }

    void QMI8658BinarySensorComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "QMI8658:");
      if (this->is_failed())
      {
        ESP_LOGE(TAG, "Communication with QMI8658 failed!");
      }
      LOG_BINARY_SENSOR("  ", "No Motion Sensor", this->no_motion_detected_sensor_);
      ESP_LOGCONFIG(TAG, "    X Threshold: %d", this->no_motion_x_threshold_);
      ESP_LOGCONFIG(TAG, "    Y Threshold: %d", this->no_motion_y_threshold_);
      ESP_LOGCONFIG(TAG, "    Z Threshold: %d", this->no_motion_z_threshold_);
      ESP_LOGCONFIG(TAG, "    Window: %d", this->no_motion_window_);
      LOG_BINARY_SENSOR("  ", "Any Motion Sensor", this->any_motion_detected_sensor_);
      ESP_LOGCONFIG(TAG, "    X Threshold: %d", this->any_motion_x_threshold_);
      ESP_LOGCONFIG(TAG, "    Y Threshold: %d", this->any_motion_y_threshold_);
      ESP_LOGCONFIG(TAG, "    Z Threshold: %d", this->any_motion_z_threshold_);
      ESP_LOGCONFIG(TAG, "    Window: %d", this->any_motion_window_);
      LOG_BINARY_SENSOR("  ", "Significant Motion Sensor", this->significant_motion_detected_sensor_);
      ESP_LOGCONFIG(TAG, "    Wait Window: %d", this->significant_motion_wait_window_);
      ESP_LOGCONFIG(TAG, "    Confirm Window: %d", this->significant_motion_confirm_window_);
    }

    float QMI8658BinarySensorComponent::get_setup_priority() const { return setup_priority::DATA; }

    void QMI8658BinarySensorComponent::loop() {
      // If we are not yet fully online...
      if (this->qmi8658_->current_state_ != QMI8658_STATE_ONLINE) {
        // ...try again next time
        return;
      }

      // If our motion interrupts are not configured...
      if (this->current_state_ != QMI8658_STATE_ONLINE) {
        //...configure them
        this->configure_motion_interrupts_();
      }


      // ESP_LOGD(TAG, "Interrupt Pin Status: %d", this->interrupt1_pin_->digital_read());
      if (this->interrupt1_pin_->digital_read() == 1) {
          // Pull the status1 register a single time so we can check the bits below
        auto status1_register_value = this->qmi8658_->status1_register_.get();
        // ESP_LOGD(TAG, "Status 1 Register is %X", status1_register_value);

        if (this->no_motion_detected_sensor_ != nullptr) {
          this->no_motion_detected_sensor_->publish_state(status1_register_value & QMI8658_STATUS1_NO_MOTION_MASK);
        }
        if (this->any_motion_detected_sensor_ != nullptr) {
          this->any_motion_detected_sensor_->publish_state(status1_register_value & QMI8658_STATUS1_ANY_MOTION_MASK);
        }
        if (this->significant_motion_detected_sensor_ != nullptr) {
          this->significant_motion_detected_sensor_->publish_state(status1_register_value & QMI8658_STATUS1_SIGNIFICANT_MOTION_MASK);
        }
      }
      return;
    }

    void QMI8658BinarySensorComponent::configure_motion_interrupts_() {
      // These will be used later when sending the significant motion window settings
      uint8_t lower, higher;

      // Configuration of motion engines should be done while accelerometer and gyro are disabled per datasheet
      // Disable Accel and Gyro
      this->qmi8658_->ctrl7_register_ &= ~(QMI8658_CTRL7_GYRO_ENABLE_MASK | QMI8658_CTRL7_ACCEL_ENABLE_MASK);

      // AnyMotionXThr
      this->qmi8658_->cal1_l_register_ = this->any_motion_x_threshold_;
      // AnyMotionYThr
      this->qmi8658_->cal1_h_register_ = this->any_motion_y_threshold_;
      // AnyMotionZThr
      this->qmi8658_->cal2_l_register_ = this->any_motion_z_threshold_;

      // AnyMotionXThr
      this->qmi8658_->cal2_h_register_ = this->no_motion_x_threshold_;
      // AnyMotionYThr
      this->qmi8658_->cal3_l_register_ = this->no_motion_y_threshold_;
      // AnyMotionZThr
      this->qmi8658_->cal3_h_register_ = this->no_motion_z_threshold_;

      //MOTION_MODE_CTRL
      // 0xF7 == NoMotion requires ALL axises to be still (AND), AnyMotion requires ANY axis to have motion (OR)
      // 0xA2 == Enable No Motion and Any Motion on X-Axis only
      this->qmi8658_->cal4_l_register_ = 0xF7;

      // Sending first configuration command
      this->qmi8658_->cal4_h_register_ = 0x01;

      // Push config
      bool success = this->qmi8658_->send_ctrl9_command_(QMI8658_CTRL9_CMD_CONFIGURE_MOTION);
      if(!success) {
        ESP_LOGD(TAG, "Failed to configure motion 1");
        this->mark_failed();
      }

      // AnyMotionWindow
      this->qmi8658_->cal1_l_register_ = this->any_motion_window_;
      //NoMotionWindow
      this->qmi8658_->cal1_h_register_ = this->no_motion_window_;

      // SigMotionWaitWindow
      splitUint16(this->significant_motion_wait_window_, lower, higher);
      this->qmi8658_->cal2_l_register_ = lower;
      this->qmi8658_->cal2_h_register_ = higher;

      // SigMotionConfirmWindow
      splitUint16(this->significant_motion_confirm_window_, lower, higher);
      this->qmi8658_->cal3_l_register_ = lower;
      this->qmi8658_->cal3_h_register_ = higher;

      // Sending second configuration command
      this->qmi8658_->cal4_h_register_ = 0x02;

      // Push config
      success = this->qmi8658_->send_ctrl9_command_(QMI8658_CTRL9_CMD_CONFIGURE_MOTION);
      if(!success) {
        ESP_LOGD(TAG, "Failed to configure motion 2");
        this->mark_failed();
      }

      // Set motion engine to use interrupt 1 pin
      this->qmi8658_->ctrl8_register_ |= QMI8658_CTRL8_ACTIVITY_INT_SELECT_MASK;

      // Enable Int1 pin
      this->qmi8658_->ctrl1_register_ |= QMI8658_CTRL1_INT1_EN_MASK;

      // Re-Enable Accel and Gyro
      this->qmi8658_->ctrl7_register_ |= QMI8658_CTRL7_GYRO_ENABLE_MASK | QMI8658_CTRL7_ACCEL_ENABLE_MASK;

      // Enable Any, No and Significant Motion engines (this might need to be done after accelerometer is enabled)
      this->qmi8658_->ctrl8_register_ |= QMI8658_CTRL8_ANY_MOTION_ENABLE_MASK | QMI8658_CTRL8_NO_MOTION_ENABLE_MASK | QMI8658_CTRL8_SIGNIFICANT_MOTION_ENABLE_MASK;

      this->current_state_ = QMI8658_STATE_ONLINE;
      
    }

  } // namespace qmi8658
} // namespace esphome
