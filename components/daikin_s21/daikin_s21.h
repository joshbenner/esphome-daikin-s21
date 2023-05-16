#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace daikin_s21 {

enum class FanMode : char {
  Auto = 'A',
  Speed1 = '3',
  Speed2 = '4',
  Speed3 = '5',
  Speed4 = '6',
  Speed5 = '7'
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

struct DaikinS21State {
  climate::ClimateMode mode = climate::CLIMATE_MODE_OFF;
  FanMode fan_mode = FanMode::Auto;
  bool swing_v = false;
  bool swing_h = false;
  bool powerful = false;
  bool econo = false;
  bool idle = false;
  // Temps stored as integer celsius * 10 instead of float.
  int16_t setpoint = 230;
  int16_t inside = 0;
  int16_t outside = 0;
  int16_t coil = 0;
  uint16_t fan_rpm = 0;
};

class DaikinS21Climate : public climate::Climate, public PollingComponent {
 public:
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;

  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx);

 protected:
  climate::ClimateTraits traits() override;

  bool read_frame(std::vector<uint8_t> &payload);
  void write_frame(std::vector<uint8_t> payload);

  bool s21_query(std::vector<uint8_t> code);
  bool parse_response(std::vector<uint8_t> rcode, std::vector<uint8_t> payload);
  void run_queries(std::vector<std::string> queries);
  void check_uart_settings();

  DaikinS21State state;
  uart::UARTComponent *tx_uart{nullptr};
  uart::UARTComponent *rx_uart{nullptr};
};

}  // namespace daikin_s21
}  // namespace esphome
