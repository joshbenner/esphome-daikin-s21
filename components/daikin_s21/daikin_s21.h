#pragma once

#include <map>
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

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

  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx);

 protected:
  climate::ClimateTraits traits() override;

  bool read_frame(std::vector<uint8_t> &payload);
  void write_frame(std::vector<uint8_t> payload);

  bool s21_query(std::vector<uint8_t> code);
  bool parse_response(std::vector<uint8_t> rcode, std::vector<uint8_t> payload);
  void run_queries(std::vector<std::string> queries);
  void check_uart_settings();

  uart::UARTComponent *tx_uart{nullptr};
  uart::UARTComponent *rx_uart{nullptr};
};

}  // namespace daikin_s21
}  // namespace esphome
