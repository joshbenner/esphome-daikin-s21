#pragma once

#include "esphome/components/sensor/sensor.h"
#include "../s21.h"

namespace esphome {
namespace daikin_s21 {

class DaikinS21Sensor : public PollingComponent, public DaikinS21Client {
 public:
  void update() override;
  void dump_config() override;

  void set_temp_inside_sensor(sensor::Sensor *sensor) {
    this->temp_inside_sensor_ = sensor;
  }
  void set_temp_outside_sensor(sensor::Sensor *sensor) {
    this->temp_outside_sensor_ = sensor;
  }
  void set_temp_coil_sensor(sensor::Sensor *sensor) {
    this->temp_coil_sensor_ = sensor;
  }
  void set_fan_speed_sensor(sensor::Sensor *sensor) {
    this->fan_speed_sensor_ = sensor;
  }

 protected:
  sensor::Sensor *temp_inside_sensor_{nullptr};
  sensor::Sensor *temp_outside_sensor_{nullptr};
  sensor::Sensor *temp_coil_sensor_{nullptr};
  sensor::Sensor *fan_speed_sensor_{nullptr};
};

}  // namespace daikin_s21
}  // namespace esphome
