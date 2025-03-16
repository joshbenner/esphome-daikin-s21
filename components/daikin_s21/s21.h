#pragma once

#include <bitset>
#include <vector>
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace daikin_s21 {

enum class DaikinClimateMode : uint8_t {
  Disabled = '0',
  Auto = '1',
  Dry = '2',
  Cool = '3',
  Heat = '4',
  Fan = '6',
};

enum class DaikinFanMode : uint8_t {
  Auto = 'A',
  Silent = 'B',
  Speed1 = '3',
  Speed2 = '4',
  Speed3 = '5',
  Speed4 = '6',
  Speed5 = '7',
};

std::string daikin_climate_mode_to_string(DaikinClimateMode mode);
std::string daikin_fan_mode_to_string(DaikinFanMode mode);

inline float c10_c(int16_t c10) { return c10 / 10.0; }
inline float c10_f(int16_t c10) { return c10_c(c10) * 1.8 + 32.0; }

struct DaikinSettings {
  bool power_on = false;
  DaikinClimateMode mode = DaikinClimateMode::Disabled;
  DaikinFanMode fan = DaikinFanMode::Auto;
  int16_t setpoint = 23;
  bool swing_v = false;
  bool swing_h = false;
};

class DaikinS21 : public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx);
  void set_debug_protocol(bool set) { this->debug_protocol = set; }

  bool is_ready() { return this->ready.all(); }

  bool is_power_on() { return this->active.power_on; }
  DaikinClimateMode get_climate_mode() { return this->active.mode; }
  DaikinFanMode get_fan_mode() { return this->active.fan; }
  float get_setpoint() { return this->active.setpoint / 10.0; }
  bool get_swing_h() { return this->active.swing_h; }
  bool get_swing_v() { return this->active.swing_v; }

  // external command actions
  void set_daikin_climate_settings(bool power_on, DaikinClimateMode mode,
                                   float setpoint, DaikinFanMode fan_mode);
  void set_swing_settings(bool swing_v, bool swing_h);

  float get_temp_inside() { return this->temp_inside / 10.0; }
  float get_temp_outside() { return this->temp_outside / 10.0; }
  float get_temp_coil() { return this->temp_coil / 10.0; }
  uint16_t get_fan_rpm() { return this->fan_rpm; }
  uint8_t get_swing_vertical_angle() { return this->swing_vertical_angle; }
  uint16_t get_compressor_frequency() { return this->compressor_hz; }
  bool is_idle() { return this->compressor_hz == 0; }

 protected:
  static constexpr uint32_t S21_RESPONSE_TURNAROUND = 75; // allow some time for the unit to begin listening after it sends
  static constexpr uint32_t S21_RESPONSE_TIMEOUT = 250; // character timeout when expecting a response from the unit
  static constexpr uint32_t S21_ERROR_TIMEOUT = 3000; // cooldown time when something goes wrong
  static constexpr uint32_t S21_MAX_COMMAND_SIZE = 4;
  static constexpr uint32_t S21_MAX_PAYLOAD_SIZE = 4;

  void dump_state();
  void check_uart_settings();
  void write_frame(const uint8_t *payload = nullptr, size_t payload_len = 0);
  void tx_next_command();
  void parse_command_response();
  void handle_rx_byte(uint8_t new_byte);

  uart::UARTComponent *tx_uart{nullptr};
  uart::UARTComponent *rx_uart{nullptr};

  // communication state machine
  enum class CommState : uint8_t {
    Idle,
    QueryAck,
    QueryStx,
    QueryEtx,
    CommandAck,
    Error
  };
  CommState comm_state = CommState::Idle;
  enum RequiredCommand : uint8_t {
    ReadyBasic,
    ReadySwing,
    ReadyCompressor,
    ReadyCount, // just for bitset sizing
  };
  std::bitset<ReadyCount> ready = {};
  std::vector<const char *> queries = {};
  std::vector<const char *>::iterator current_query;
  const char *tx_command = "";  // used when matching responses - value must have persistent lifetime
  std::vector<uint8_t> rx_buffer = {};
  uint32_t rx_timeout = 0;
  bool refresh_state = false;
  bool debug_protocol = false;
  std::unordered_map<std::string, std::vector<uint8_t>> val_cache;  // debugging

  // settings
  DaikinSettings active = {};
  DaikinSettings pending = {};
  bool activate_climate = false;
  bool activate_swing_mode = false;

  // current values
  int16_t temp_inside = 0;
  int16_t temp_outside = 0;
  int16_t temp_coil = 0;
  uint16_t fan_rpm = 0;
  int16_t swing_vertical_angle = 0;
  uint8_t compressor_hz = 0;

  //protocol support
  bool support_rg = false;
};

class DaikinS21Client {
 public:
  void set_s21(DaikinS21 *s21) { this->s21 = s21; }

 protected:
  DaikinS21 *s21;
};

}  // namespace daikin_s21
}  // namespace esphome
