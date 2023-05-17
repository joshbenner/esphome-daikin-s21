#pragma once

#include <map>
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "../s21.h"

namespace esphome {
namespace daikin_s21 {

static std::map<uint8_t, std::string> FanModes = {
  {'A', "Auto"}, {'3', "1"}, {'4', "2"}, {'5', "3"}, {'6', "4"}, {'7', "5"}
};

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

class DaikinS21Climate : public climate::Climate, public PollingComponent {
 public:
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;
  void set_s21(DaikinS21 *s21) { this->s21 = s21; }

  climate::ClimateAction d2e_climate_action();

  climate::ClimateMode d2e_climate_mode(DaikinClimateMode mode);
  DaikinClimateMode e2d_climate_mode(climate::ClimateMode mode);
  const std::string d2e_fan_mode(DaikinFanMode mode);
  DaikinFanMode e2d_fan_mode(std::string mode);
  climate::ClimateSwingMode d2e_swing_mode(bool swing_v, bool swing_h);
  bool e2d_swing_v(climate::ClimateSwingMode mode);
  bool e2d_swing_h(climate::ClimateSwingMode mode);

 protected:
  climate::ClimateTraits traits() override;

  DaikinS21 *s21;
};

}  // namespace daikin_s21
}  // namespace esphome
