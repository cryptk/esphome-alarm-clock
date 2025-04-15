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
      LOG_BINARY_SENSOR("  ", "Any Motion Sensor", this->any_motion_detected_sensor_);
      LOG_BINARY_SENSOR("  ", "Significant Motion Sensor", this->significant_motion_detected_sensor_);
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
      // TODO: This should be togglable and configurable functionality

      // Determines how much motion there must be
      // This is a difference in value between subsequent samples measured in units of 1/32nd of a g of acceleration
      auto AnyMotionXThr = 0x03;
      auto AnyMotionYThr = 0x03;
      auto AnyMotionZThr = 0x03;
      // measured in number of samples based on the Accelerometer Output Data Rate
      // 0x0A == 10
      auto AnyMotionWindow = 0x03;

      auto NoMotionXThr = 0x01;
      auto NoMotionYThr = 0x01;
      auto NoMotionZThr = 0x01;
      // measured in number of samples based on the Accelerometer Output Data Rate
      // 0xC8 == 200
      auto NoMotionWindow = 0xFF;

      auto CTRL9_CMD_CONFIGURE_MOTION = 0x0E;
      auto CTRL9_CMD_ACK = 0x00;

      // Configuration of motion engines should be done while accelerometer and gyro are disabled per datasheet
      // Disable Accel and Gyro
      this->qmi8658_->ctrl7_register_ &= 0xFC;

      // ESP_LOGD(TAG, "Register status: %d", this->cal1_l_register_.get());
      // range 0-255, 0x80 == 128
      // AnyMotionXThr
      this->qmi8658_->cal1_l_register_ = AnyMotionXThr;
      // AnyMotionYThr
      this->qmi8658_->cal1_h_register_ = AnyMotionYThr;
      // AnyMotionZThr
      this->qmi8658_->cal2_l_register_ = AnyMotionZThr;

      // range 0-255, 0x40 == 64
      // AnyMotionXThr
      this->qmi8658_->cal2_h_register_ = NoMotionXThr;
      // AnyMotionYThr
      this->qmi8658_->cal3_l_register_ = NoMotionYThr;
      // AnyMotionZThr
      this->qmi8658_->cal3_h_register_ = NoMotionZThr;

      //MOTION_MODE_CTRL
      // 0xF7 == NoMotion requires ALL axises to be still (AND), AnyMotion requires ANY axis to have motion (OR)
      // 0xA2 == Enable No Motion and Any Motion on X-Axis only
      this->qmi8658_->cal4_l_register_ = 0xF7;

      // Sending first configuration command
      this->qmi8658_->cal4_h_register_ = 0x01;

      // Push config
      this->qmi8658_->ctrl9_register_ = CTRL9_CMD_CONFIGURE_MOTION;
      delay(10);
      if (this->qmi8658_->statusint_register_.get() & 0x80 != 0x80) {
        ESP_LOGE(TAG, "Configure Motion Command 1 Failure");
      }
      ESP_LOGD(TAG, "Configure Motion Command 1 Success");
      this->qmi8658_->ctrl9_register_ = CTRL9_CMD_ACK;
      delay(10);
      if (this->qmi8658_->statusint_register_.get() & 0x80 != 0x00) {
        ESP_LOGE(TAG, "QMI Failed to clear STATUSINT after Command 1");
      }

      // AnyMotionWindow
      // TODO: Validate that these registers are zeroed out after the CTRL_CMD_CONFIGURE_MOTION command above before writing to them again
      // ESP_LOGD(TAG, "Register status: %d", this->cal1_l_register_.get());
      this->qmi8658_->cal1_l_register_ = AnyMotionWindow;

      //NoMotionWindow
      this->qmi8658_->cal1_h_register_ = NoMotionWindow;

      // SigMotionWaitWindow == 100 samples
      this->qmi8658_->cal2_l_register_ = 0x64;
      this->qmi8658_->cal2_h_register_ = 0x00;

      // SigMotionConfirmWindow == 65535 samples
      this->qmi8658_->cal3_l_register_ = 0xFF;
      this->qmi8658_->cal3_h_register_ = 0xFF;

      // Sending second configuration command
      this->qmi8658_->cal4_h_register_ = 0x02;

      // Push config
      this->qmi8658_->ctrl9_register_ = CTRL9_CMD_CONFIGURE_MOTION;
      delay(10);
      if (this->qmi8658_->statusint_register_.get() & 0x80 != 0x80) {
        ESP_LOGE(TAG, "Configure Motion Command 2 Failure");
      }
      ESP_LOGD(TAG, "Configure Motion Command 2 Success");
      this->qmi8658_->ctrl9_register_ = CTRL9_CMD_ACK;
      delay(10);
      if (this->qmi8658_->statusint_register_.get() & 0x80 != 0x00) {
        ESP_LOGE(TAG, "QMI Failed to clear STATUSINT after Command 2");
      }

      // Set motion engine to use interrupt 1 pin
      this->qmi8658_->ctrl8_register_ |= 0x40;

      // Enable Int1 pin
      this->qmi8658_->ctrl1_register_ |= 0x08;

      // Re-Enable Accel and Gyro
      this->qmi8658_->ctrl7_register_ |= 0x03;

      // Enable Any, No and Significant Motion engines (this might need to be done after accelerometer is enabled)
      this->qmi8658_->ctrl8_register_ |= 0x0E;

      this->current_state_ = QMI8658_STATE_ONLINE;
      
    }

  } // namespace qmi8658
} // namespace esphome
