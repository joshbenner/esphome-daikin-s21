#include "daikin_s21_sensor.h"

namespace esphome {
namespace daikin_s21 {

static const char *const TAG = "daikin_s21.sensor";

void DaikinS21Sensor::update() {
  if (!this->s21->is_ready())
    return;
  if (this->temp_inside_sensor_ != nullptr) {
    this->temp_inside_sensor_->publish_state(this->s21->get_temp_inside());
  }
  if (this->temp_outside_sensor_ != nullptr) {
    this->temp_outside_sensor_->publish_state(this->s21->get_temp_outside());
  }
  if (this->temp_coil_sensor_ != nullptr) {
    this->temp_coil_sensor_->publish_state(this->s21->get_temp_coil());
  }
  if (this->fan_speed_sensor_ != nullptr) {
    this->fan_speed_sensor_->publish_state(this->s21->get_fan_rpm());
  }
}

void DaikinS21Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Sensor:");
  LOG_SENSOR("  ", "Temperature Inside", this->temp_inside_sensor_);
  LOG_SENSOR("  ", "Temperature Outside", this->temp_outside_sensor_);
  LOG_SENSOR("  ", "Temperature Coil", this->temp_coil_sensor_);
  LOG_SENSOR("  ", "Fan Speed", this->fan_speed_sensor_);
}

}  // namespace daikin_s21
}  // namespace esphome
