#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "../qmi8658.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome
{
  namespace qmi8658
  {

    class QMI8658BinarySensorComponent : public Component
    {
    public:
      void setup() override;
      void dump_config() override;

      void loop() override;

      float get_setup_priority() const override;

      void set_no_motion_detected_sensor(binary_sensor::BinarySensor *no_motion_detected_sensor) { no_motion_detected_sensor_ = no_motion_detected_sensor; };
      void set_any_motion_detected_sensor(binary_sensor::BinarySensor *any_motion_detected_sensor) { any_motion_detected_sensor_ = any_motion_detected_sensor; };
      void set_significant_motion_detected_sensor(binary_sensor::BinarySensor *significant_motion_detected_sensor) { significant_motion_detected_sensor_ = significant_motion_detected_sensor; };

      void set_interrupt1_pin(GPIOPin *interrupt1_pin) {interrupt1_pin_ = interrupt1_pin; };
      void set_interrupt2_pin(GPIOPin *interrupt2_pin) {interrupt2_pin_ = interrupt2_pin; };

      void set_parent(QMI8658Component *qmi8658) {this->qmi8658_ = qmi8658;};
      
    protected:

      QMI8658Component *qmi8658_{nullptr};
      
      binary_sensor::BinarySensor *no_motion_detected_sensor_{nullptr};
      binary_sensor::BinarySensor *any_motion_detected_sensor_{nullptr};
      binary_sensor::BinarySensor *significant_motion_detected_sensor_{nullptr};

      GPIOPin *interrupt1_pin_{nullptr};
      // void handle_interrupt1_();
      GPIOPin *interrupt2_pin_{nullptr};
      // void handle_interrupt2_();

      void configure_motion_interrupts_();

      qmi8658_state_t current_state_ = QMI8658_STATE_UNKNOWN;
    };
    ;

  } // namespace qmi8658
} // namespace esphome
