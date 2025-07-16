#pragma once

#include <map>
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "../s21.h"

namespace esphome {
namespace daikin_s21 {

// clang-format off
static const climate::ClimateMode OpModes[] = {
    climate::CLIMATE_MODE_OFF,  // Unused
    climate::CLIMATE_MODE_HEAT_COOL,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_OFF,  // Unused
    climate::CLIMATE_MODE_FAN_ONLY
};
// clang-format on

class DaikinS21Climate : public climate::Climate,
                         public PollingComponent,
                         public DaikinS21Client {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;

  void set_room_sensor(sensor::Sensor *sensor) { this->room_sensor_ = sensor; }
  void set_setpoint_interval(uint16_t seconds) {
    this->setpoint_interval = seconds;
  };
  float get_s21_setpoint() { return this->s21->get_setpoint(); }
  float get_room_temp_offset();

  bool should_check_setpoint(climate::ClimateMode mode);
  climate::ClimateAction d2e_climate_action();
  climate::ClimateMode d2e_climate_mode(DaikinClimateMode mode);
  DaikinClimateMode e2d_climate_mode(climate::ClimateMode mode);
  const std::string d2e_fan_mode(DaikinFanMode mode);
  DaikinFanMode e2d_fan_mode(std::string mode);
  climate::ClimateSwingMode d2e_swing_mode(bool swing_v, bool swing_h);
  bool e2d_swing_v(climate::ClimateSwingMode mode);
  bool e2d_swing_h(climate::ClimateSwingMode mode);

  void set_supported_modes(const std::set<esphome::climate::ClimateMode> &modes);

 protected:
  esphome::climate::ClimateTraits traits_;

  sensor::Sensor *room_sensor_{nullptr};
  float expected_s21_setpoint;
  uint8_t skip_setpoint_checks = 0;
  uint16_t setpoint_interval = 0;
  uint32_t last_setpoint_check = 0;


  ESPPreferenceObject auto_setpoint_pref;
  ESPPreferenceObject cool_setpoint_pref;
  ESPPreferenceObject heat_setpoint_pref;

  climate::ClimateTraits traits() override;

  bool use_room_sensor();
  bool room_sensor_unit_is_valid();
  float room_sensor_degc();
  float get_effective_current_temperature();
  float calc_s21_setpoint(float target);
  float s21_setpoint_variance();
  void save_setpoint(float value, ESPPreferenceObject &pref);
  void save_setpoint(float value);
  optional<float> load_setpoint(ESPPreferenceObject &pref);
  optional<float> load_setpoint(DaikinClimateMode mode);
  void set_s21_climate();
};

}  // namespace daikin_s21
}  // namespace esphome
