#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "climate.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

#define SETPOINT_MIN 18
#define SETPOINT_MAX 32
#define SETPOINT_STEP 0.5f

static const char *const TAG = "daikin_s21.climate";

void DaikinS21Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21Climate:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
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

  std::set<std::string> fan_mode_names;
  for (auto m : FanModes) {
    fan_mode_names.insert(m.second);
  }
  traits.set_supported_custom_fan_modes(fan_mode_names);

  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });

  return traits;
}

void DaikinS21Climate::set_daikin_climate_mode(DaikinClimateMode mode) {
  switch (mode) {
    case DaikinClimateMode::Auto:
      this->mode = climate::CLIMATE_MODE_AUTO;
      break;
    case DaikinClimateMode::Cool:
      this->mode = climate::CLIMATE_MODE_COOL;
      break;
    case DaikinClimateMode::Heat:
      this->mode = climate::CLIMATE_MODE_HEAT;
      break;
    case DaikinClimateMode::Dry:
      this->mode = climate::CLIMATE_MODE_DRY;
      break;
    case DaikinClimateMode::Fan:
      this->mode = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    // case DaikinClimateMode::Disabled:
    // default:
      // Do nothing
  }
}

void DaikinS21Climate::set_daikin_fan_mode(DaikinFanMode mode) {
  switch (mode) {
    case DaikinFanMode::Auto:
      this->set_custom_fan_mode_("Auto");
      break;
    case DaikinFanMode::Speed1:
      this->set_custom_fan_mode_("1");
      break;
    case DaikinFanMode::Speed2:
      this->set_custom_fan_mode_("2");
      break;
    case DaikinFanMode::Speed3:
      this->set_custom_fan_mode_("3");
      break;
    case DaikinFanMode::Speed4:
      this->set_custom_fan_mode_("4");
      break;
    case DaikinFanMode::Speed5:
      this->set_custom_fan_mode_("5");
      break;
    // default:
      // Do nothing
  }
}

void DaikinS21Climate::update() {
  this->set_daikin_climate_mode(this->s21->get_climate_mode());
}

void DaikinS21Climate::control(const climate::ClimateCall &call) {}

}  // namespace daikin_s21
}  // namespace esphome
