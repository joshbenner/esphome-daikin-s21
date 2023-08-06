#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "daikin_s21_climate.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

#define SETPOINT_MIN 18
#define SETPOINT_MAX 32
#define SETPOINT_STEP 0.5f

static const char *const TAG = "daikin_s21.climate";

void DaikinS21Climate::setup() {
  uint32_t h = this->get_object_id_hash();
  auto_setpoint_pref = global_preferences->make_preference<int16_t>(h + 1);
  cool_setpoint_pref = global_preferences->make_preference<int16_t>(h + 2);
  heat_setpoint_pref = global_preferences->make_preference<int16_t>(h + 3);
}

void DaikinS21Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21Climate:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
  if (this->room_sensor_ != nullptr) {
    if (!this->room_sensor_unit_is_valid()) {
      ESP_LOGCONFIG(TAG, "  ROOM SENSOR: INVALID UNIT '%s' (must be °C or °F)",
                    this->room_sensor_->get_unit_of_measurement().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Room sensor: %s",
                    this->room_sensor_->get_name().c_str());
      ESP_LOGCONFIG(TAG, "  Setpoint interval: %d", this->setpoint_interval);
    }
  }
  this->dump_traits_(TAG);
}

climate::ClimateTraits DaikinS21Climate::traits() {
  auto traits = climate::ClimateTraits();

  traits.set_supports_action(true);

  traits.set_supports_current_temperature(true);
  traits.set_visual_min_temperature(SETPOINT_MIN);
  traits.set_visual_max_temperature(SETPOINT_MAX);
  traits.set_visual_temperature_step(SETPOINT_STEP);
  traits.set_supports_two_point_target_temperature(false);

  traits.set_supported_modes(
      {climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT_COOL,
       climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_HEAT,
       climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_DRY});

  traits.set_supported_custom_fan_modes({"Automatic", "Silent", "1", "2", "3", "4", "5"});

  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });

  return traits;
}

bool DaikinS21Climate::use_room_sensor() {
  return this->room_sensor_unit_is_valid() && this->room_sensor_->has_state() &&
         !isnanf(this->room_sensor_->get_state());
}

bool DaikinS21Climate::room_sensor_unit_is_valid() {
  if (this->room_sensor_ != nullptr) {
    auto u = this->room_sensor_->get_unit_of_measurement();
    return u == "°C" || u == "°F";
  }
  return false;
}

float DaikinS21Climate::room_sensor_degc() {
  float temp = this->room_sensor_->get_state();
  if (this->room_sensor_->get_unit_of_measurement() == "°F") {
    temp = fahrenheit_to_celsius(temp);
  }
  return temp;
}

float DaikinS21Climate::get_effective_current_temperature() {
  if (this->use_room_sensor()) {
    return this->room_sensor_degc();
  }
  return this->s21->get_temp_inside();
}

float DaikinS21Climate::get_room_temp_offset() {
  if (!this->use_room_sensor()) {
    return 0.0;
  }
  float room_val = this->room_sensor_degc();
  float s21_val = this->s21->get_temp_inside();
  return s21_val - room_val;
}

float nearest_step(float temp) {
  return std::round(temp / SETPOINT_STEP) * SETPOINT_STEP;
}

// What setpoint should be sent to s21, acconting for external room sensor.
float DaikinS21Climate::calc_s21_setpoint(float target) {
  float offset_target = target + this->get_room_temp_offset();
  return nearest_step(offset_target);
}

// How far from desired setpoint is the current S21 setpoint?
float DaikinS21Climate::s21_setpoint_variance() {
  return abs(this->s21->get_setpoint() -
             this->calc_s21_setpoint(this->target_temperature));
}

void DaikinS21Climate::save_setpoint(float value, ESPPreferenceObject &pref) {
  int16_t stored_val = static_cast<int16_t>(value * 10.0);
  pref.save(&stored_val);
}

void DaikinS21Climate::save_setpoint(float value) {
  auto mode = this->s21->get_climate_mode();
  optional<float> prev = this->load_setpoint(mode);
  // Only save if value is diff from what's already saved.
  if (abs(value - prev.value_or(0.0)) >= SETPOINT_STEP) {
    switch (mode) {
      case DaikinClimateMode::Auto:
        this->save_setpoint(value, this->auto_setpoint_pref);
        break;
      case DaikinClimateMode::Cool:
        this->save_setpoint(value, this->cool_setpoint_pref);
        break;
      case DaikinClimateMode::Heat:
        this->save_setpoint(value, this->heat_setpoint_pref);
        break;
    }
  }
}

optional<float> DaikinS21Climate::load_setpoint(ESPPreferenceObject &pref) {
  int16_t stored_val = 0;
  if (!pref.load(&stored_val)) {
    return {};
  }
  return static_cast<float>(stored_val) / 10.0;
}

optional<float> DaikinS21Climate::load_setpoint(DaikinClimateMode mode) {
  optional<float> loaded;
  switch (this->s21->get_climate_mode()) {
    case DaikinClimateMode::Auto:
      loaded = this->load_setpoint(this->auto_setpoint_pref);
      break;
    case DaikinClimateMode::Cool:
      loaded = this->load_setpoint(this->cool_setpoint_pref);
      break;
    case DaikinClimateMode::Heat:
      loaded = this->load_setpoint(this->heat_setpoint_pref);
      break;
  }
  return loaded;
}

bool DaikinS21Climate::should_check_setpoint(climate::ClimateMode mode) {
  bool mode_uses_setpoint = mode == climate::CLIMATE_MODE_AUTO ||
                            mode == climate::CLIMATE_MODE_COOL ||
                            mode == climate::CLIMATE_MODE_HEAT ||
                            mode == climate::CLIMATE_MODE_HEAT_COOL;
  bool skip_check = false;
  if (this->skip_setpoint_checks > 0) {
    this->skip_setpoint_checks--;
    skip_check = true;
  }
  bool min_passed =
      this->setpoint_interval == 0 || this->last_setpoint_check == 0 ||
      (millis() - this->last_setpoint_check > (this->setpoint_interval * 1000));
  return mode_uses_setpoint & !skip_check && min_passed;
}

climate::ClimateMode DaikinS21Climate::d2e_climate_mode(
    DaikinClimateMode mode) {
  switch (mode) {
    case DaikinClimateMode::Auto:
      return climate::CLIMATE_MODE_HEAT_COOL;
    case DaikinClimateMode::Cool:
      return climate::CLIMATE_MODE_COOL;
    case DaikinClimateMode::Heat:
      return climate::CLIMATE_MODE_HEAT;
    case DaikinClimateMode::Dry:
      return climate::CLIMATE_MODE_DRY;
    case DaikinClimateMode::Fan:
      return climate::CLIMATE_MODE_FAN_ONLY;
    case DaikinClimateMode::Disabled:
    default:
      return climate::CLIMATE_MODE_OFF;
  }
}

DaikinClimateMode DaikinS21Climate::e2d_climate_mode(
    climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT_COOL:
      return DaikinClimateMode::Auto;
    case climate::CLIMATE_MODE_HEAT:
      return DaikinClimateMode::Heat;
    case climate::CLIMATE_MODE_COOL:
      return DaikinClimateMode::Cool;
    case climate::CLIMATE_MODE_DRY:
      return DaikinClimateMode::Dry;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return DaikinClimateMode::Fan;
    default:
      return DaikinClimateMode::Disabled;
  }
}

const std::string DaikinS21Climate::d2e_fan_mode(DaikinFanMode mode) {
  switch (mode) {
    case DaikinFanMode::Speed1:
      return "1";
    case DaikinFanMode::Speed2:
      return "2";
    case DaikinFanMode::Speed3:
      return "3";
    case DaikinFanMode::Speed4:
      return "4";
    case DaikinFanMode::Speed5:
      return "5";
    case DaikinFanMode::Silent:
      return "Silent";
    case DaikinFanMode::Auto:
    default:
      return "Automatic";
  }
}

DaikinFanMode DaikinS21Climate::e2d_fan_mode(std::string mode) {
  if (mode == "Automatic")
    return DaikinFanMode::Auto;
  if (mode == "Silent")
    return DaikinFanMode::Silent;
  if (mode == "1")
    return DaikinFanMode::Speed1;
  if (mode == "2")
    return DaikinFanMode::Speed2;
  if (mode == "3")
    return DaikinFanMode::Speed3;
  if (mode == "4")
    return DaikinFanMode::Speed4;
  if (mode == "5")
    return DaikinFanMode::Speed5;
  return DaikinFanMode::Auto;
}

climate::ClimateAction DaikinS21Climate::d2e_climate_action() {
  if (this->s21->is_idle()) {
    return climate::CLIMATE_ACTION_IDLE;
  }
  switch (this->s21->get_climate_mode()) {
    uint16_t setpoint, temp_inside;

    case DaikinClimateMode::Auto:
      setpoint = this->s21->get_setpoint();
      temp_inside = this->s21->get_temp_inside();
      if (setpoint > temp_inside) {
        return climate::CLIMATE_ACTION_HEATING;
      } else if (setpoint < temp_inside) {
        return climate::CLIMATE_ACTION_COOLING;
      }
      return climate::CLIMATE_ACTION_IDLE;
    case DaikinClimateMode::Cool:
      return climate::CLIMATE_ACTION_COOLING;
    case DaikinClimateMode::Heat:
      return climate::CLIMATE_ACTION_HEATING;
    case DaikinClimateMode::Dry:
      return climate::CLIMATE_ACTION_DRYING;
    case DaikinClimateMode::Fan:
      return climate::CLIMATE_ACTION_FAN;
    default:
      return climate::CLIMATE_ACTION_OFF;
  }
}

climate::ClimateSwingMode DaikinS21Climate::d2e_swing_mode(bool swing_v,
                                                           bool swing_h) {
  if (swing_v && swing_h)
    return climate::CLIMATE_SWING_BOTH;
  if (swing_v)
    return climate::CLIMATE_SWING_VERTICAL;
  if (swing_h)
    return climate::CLIMATE_SWING_HORIZONTAL;
  return climate::CLIMATE_SWING_OFF;
}

bool DaikinS21Climate::e2d_swing_h(climate::ClimateSwingMode mode) {
  return mode == climate::CLIMATE_SWING_BOTH ||
         mode == climate::CLIMATE_SWING_HORIZONTAL;
}

bool DaikinS21Climate::e2d_swing_v(climate::ClimateSwingMode mode) {
  return mode == climate::CLIMATE_SWING_BOTH ||
         mode == climate::CLIMATE_SWING_VERTICAL;
}

void DaikinS21Climate::update() {
  if (this->use_room_sensor()) {
    ESP_LOGD(TAG, "Room temp from external sensor: %.1f %s (%.1f °C)",
             this->room_sensor_->get_state(),
             this->room_sensor_->get_unit_of_measurement().c_str(),
             this->room_sensor_degc());
    ESP_LOGD(TAG, "  Offset: %.1f", this->get_room_temp_offset());
  }
  if (this->s21->is_ready()) {
    if (this->s21->is_power_on()) {
      this->mode = this->d2e_climate_mode(this->s21->get_climate_mode());
      this->action = this->d2e_climate_action();
    } else {
      this->mode = climate::CLIMATE_MODE_OFF;
      this->action = climate::CLIMATE_ACTION_OFF;
    }
    this->set_custom_fan_mode_(this->d2e_fan_mode(this->s21->get_fan_mode()));
    this->swing_mode = this->d2e_swing_mode(this->s21->get_swing_v(),
                                            this->s21->get_swing_h());
    this->current_temperature = this->get_effective_current_temperature();

    if (this->should_check_setpoint(this->mode)) {
      this->last_setpoint_check = millis();
      // Target temperature is stored by climate class, and is used to represent
      // the user's desired temperature. This is distinct from the HVAC unit's
      // setpoint because we may be using an external sensor. So we only update
      // the target temperature here if it appears uninitialized.
      float current_s21_sp = this->s21->get_setpoint();
      float unexpected_diff = abs(this->expected_s21_setpoint - current_s21_sp);
      if (this->target_temperature == 0.0) {
        // Use stored setpoint for mode, or fall back to use s21's setpoint.
        auto stored = this->load_setpoint(this->s21->get_climate_mode());
        this->target_temperature = stored.value_or(current_s21_sp);
        this->set_s21_climate();
      } else if (unexpected_diff >= SETPOINT_STEP) {
        // User probably set temp via IR remote -- so try to honor their wish by
        // matching controller's target value to what they sent via remote.
        ESP_LOGI(TAG, "S21 setpoint changed outside controller");
        ESP_LOGI(TAG, "  Expected: %.1f", this->expected_s21_setpoint);
        ESP_LOGI(TAG, "  Found: %.1f", current_s21_sp);
        this->target_temperature = current_s21_sp;
        ESP_LOGI(TAG, "  Target temp updated to %.1f", current_s21_sp);
        this->set_s21_climate();
      } else if (this->s21_setpoint_variance() >= SETPOINT_STEP) {
        // Room temperature offset has probably changed, so we need to adjust
        // the s21 setpoint based on the new difference.
        this->set_s21_climate();
        ESP_LOGI(TAG, "S21 setpoint updated to %.1f",
                 this->expected_s21_setpoint);
      }
    }

    this->publish_state();
  }
}

void DaikinS21Climate::control(const climate::ClimateCall &call) {
  float setpoint = this->target_temperature;
  std::string fan_mode = this->custom_fan_mode.value_or("Automatic");
  bool set_basic = false;

  if (call.get_mode().has_value()) {
    climate::ClimateMode climate_mode = call.get_mode().value();
    if (this->mode != climate_mode) {
      this->mode = climate_mode;
      set_basic = true;
      if (!call.get_target_temperature().has_value()) {
        // If call does not include target, then try to use saved target.
        DaikinClimateMode dmode = this->e2d_climate_mode(this->mode);
        optional<float> sp = this->load_setpoint(dmode);
        if (sp.has_value()) {
          this->target_temperature = nearest_step(sp.value());
        }
      }
    }
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature =
        nearest_step(call.get_target_temperature().value());
    set_basic = true;
  }
  if (call.get_custom_fan_mode().has_value()) {
    this->custom_fan_mode = call.get_custom_fan_mode().value();
    set_basic = true;
  }

  if (set_basic) {
    this->set_s21_climate();
  }

  if (call.get_swing_mode().has_value()) {
    climate::ClimateSwingMode swing_mode = call.get_swing_mode().value();
    this->s21->set_swing_settings(this->e2d_swing_v(swing_mode),
                                  this->e2d_swing_h(swing_mode));
  }

  this->update();
}

void DaikinS21Climate::set_s21_climate() {
  this->expected_s21_setpoint =
      this->calc_s21_setpoint(this->target_temperature);
  ESP_LOGI(TAG, "Controlling S21 climate:");
  ESP_LOGI(TAG, "  Mode: %s", climate::climate_mode_to_string(this->mode));
  ESP_LOGI(TAG, "  Setpoint: %.1f (s21: %.1f)", this->target_temperature,
           this->expected_s21_setpoint);
  ESP_LOGI(TAG, "  Fan: %s", this->custom_fan_mode.value().c_str());
  this->s21->set_daikin_climate_settings(
      this->mode != climate::CLIMATE_MODE_OFF,
      this->e2d_climate_mode(this->mode), this->expected_s21_setpoint,
      this->e2d_fan_mode(this->custom_fan_mode.value()));
  // HVAC unit seems to take a few seconds to begin reporting mode and setpoint
  // changes back to the controller, so when modifying settings, setpoint checks
  // are skipped to avoid unexpected setpoint updates, especially when changing
  // modes.
  this->skip_setpoint_checks = 2;
  this->save_setpoint(this->target_temperature);
}

}  // namespace daikin_s21
}  // namespace esphome
