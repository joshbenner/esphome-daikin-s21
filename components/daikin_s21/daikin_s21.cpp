#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "daikin_s21.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

// #define S21_EXPERIMENTS

#define SETPOINT_MIN 18
#define SETPOINT_MAX 32
#define SETPOINT_STEP 0.5f

#define STX 2
#define ETX 3
#define ACK 6
#define NAK 21

#define S21_RESPONSE_TIMEOUT 250

static const char *const TAG = "ds21";

uint8_t s21_checksum(uint8_t *bytes, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum += bytes[i];
  }
  return checksum;
}

uint8_t s21_checksum(std::vector<uint8_t> bytes) {
  return s21_checksum(&bytes[0], bytes.size());
}

uint16_t bytes_to_num(uint8_t *bytes, size_t len) {
  // <ones><tens><hundreds><neg/pos>
  uint16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if (len > 3 && bytes[3] == '-')
    val *= -1;
  return val;
}

uint16_t bytes_to_num(std::vector<uint8_t> &bytes) {
  return bytes_to_num(&bytes[0], bytes.size());
}

uint16_t temp_bytes_to_c10(uint8_t *bytes) { return bytes_to_num(bytes, 4); }

uint16_t temp_bytes_to_c10(std::vector<uint8_t> &bytes) {
  return temp_bytes_to_c10(&bytes[0]);
}

inline float c10_c(uint16_t c10) { return c10 / 10.0; }

inline float c10_f(uint16_t c10) { return c10_c(c10) * 1.8 + 32.0; }

void dump_s21_state(DaikinS21State *state) {
  ESP_LOGD(TAG, "Current S21 State:");
  ESP_LOGD(TAG, "   Mode: %s (%s)",
           LOG_STR_ARG(climate::climate_mode_to_string(state->mode)),
           state->idle ? "idle" : "active");
  float degc = state->setpoint / 10.0;
  float degf = degc * 1.8 + 32.0;
  ESP_LOGD(TAG, "   Temp: %.1f C (%.1f F)", c10_c(state->setpoint),
           c10_f(state->setpoint));
  ESP_LOGD(TAG, "    Fan: %c (%d rpm)", (int)state->fan_mode, state->fan_rpm);
  ESP_LOGD(TAG, "  Swing: Vertical: %s  |  Horizontal: %s",
           state->swing_v ? "YES" : "NO", state->swing_h ? "YES" : "NO");
  ESP_LOGD(TAG, " Powerf: %s", state->powerful ? "YES" : "NO");
  ESP_LOGD(TAG, "  Econo: %s", state->econo ? "YES" : "NO");
  ESP_LOGD(TAG, " Inside: %.1f C (%.1f F)", c10_c(state->inside),
           c10_f(state->inside));
  ESP_LOGD(TAG, "Outside: %.1f C (%.1f F)", c10_c(state->outside),
           c10_f(state->outside));
  ESP_LOGD(TAG, "   Coil: %.1f C (%.1f F)", c10_c(state->coil),
           c10_f(state->coil));
}

void DaikinS21Climate::set_uarts(uart::UARTComponent *tx,
                                 uart::UARTComponent *rx) {
  this->tx_uart = tx;
  this->rx_uart = rx;
}

#define S21_BAUD_RATE 2400
#define S21_STOP_BITS 2
#define S21_DATA_BITS 8
#define S21_PARITY uart::UART_CONFIG_PARITY_EVEN

void DaikinS21Climate::check_uart_settings() {
  for (auto uart : {this->tx_uart, this->rx_uart}) {
    if (uart->get_baud_rate() != S21_BAUD_RATE) {
      ESP_LOGE(
          TAG,
          "  Invalid baud_rate: Integration requested baud_rate %u but you "
          "have %u!",
          S21_BAUD_RATE, uart->get_baud_rate());
    }
    if (uart->get_stop_bits() != S21_STOP_BITS) {
      ESP_LOGE(
          TAG,
          "  Invalid stop bits: Integration requested stop_bits %u but you "
          "have %u!",
          S21_STOP_BITS, uart->get_stop_bits());
    }
    if (uart->get_data_bits() != S21_DATA_BITS) {
      ESP_LOGE(TAG,
               "  Invalid number of data bits: Integration requested %u data "
               "bits but you have %u!",
               S21_DATA_BITS, uart->get_data_bits());
    }
    if (uart->get_parity() != S21_PARITY) {
      ESP_LOGE(
          TAG,
          "  Invalid parity: Integration requested parity %s but you have %s!",
          LOG_STR_ARG(parity_to_str(S21_PARITY)),
          LOG_STR_ARG(parity_to_str(uart->get_parity())));
    }
  }
}

void DaikinS21Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21Climate:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
  this->dump_traits_(TAG);
  this->check_uart_settings();
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

  traits.set_supported_custom_fan_modes({"Auto", "1", "2", "3", "4", "5"});

  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });

  return traits;
}

// Adapated from ESPHome UART debugger
std::string hex_repr(uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (i > 0)
      res += ':';
    sprintf(buf, "%02X", bytes[i]);
    res += buf;
  }
  return res;
}

// Adapated from ESPHome UART debugger
std::string str_repr(uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] == 7) {
      res += "\\a";
    } else if (bytes[i] == 8) {
      res += "\\b";
    } else if (bytes[i] == 9) {
      res += "\\t";
    } else if (bytes[i] == 10) {
      res += "\\n";
    } else if (bytes[i] == 11) {
      res += "\\v";
    } else if (bytes[i] == 12) {
      res += "\\f";
    } else if (bytes[i] == 13) {
      res += "\\r";
    } else if (bytes[i] == 27) {
      res += "\\e";
    } else if (bytes[i] == 34) {
      res += "\\\"";
    } else if (bytes[i] == 39) {
      res += "\\'";
    } else if (bytes[i] == 92) {
      res += "\\\\";
    } else if (bytes[i] < 32 || bytes[i] > 127) {
      sprintf(buf, "\\x%02X", bytes[i]);
      res += buf;
    } else {
      res += bytes[i];
    }
  }
  return res;
}

std::string str_repr(std::vector<uint8_t> &bytes) {
  return str_repr(&bytes[0], bytes.size());
}

bool DaikinS21Climate::read_frame(std::vector<uint8_t> &payload) {
  uint8_t byte;
  std::vector<uint8_t> bytes;
  uint32_t start = millis();
  bool reading = false;
  while (true) {
    if (millis() - start > S21_RESPONSE_TIMEOUT) {
      ESP_LOGW(TAG, "Timeout waiting for frame");
      return false;
    }
    while (this->rx_uart->available()) {
      this->rx_uart->read_byte(&byte);
      if (byte == ACK) {
        ESP_LOGW(TAG, "Unexpected ACK waiting to read start of frame");
        continue;
      } else if (!reading && byte != STX) {
        ESP_LOGW(TAG, "Unexpected byte waiting to read start of frame: %x",
                 byte);
        continue;
      } else if (byte == STX) {
        reading = true;
        continue;
      }
      if (byte == ETX) {
        reading = false;
        uint8_t frame_csum = bytes[bytes.size() - 1];
        bytes.pop_back();
        uint8_t calc_csum = s21_checksum(bytes);
        if (calc_csum != frame_csum) {
          ESP_LOGW(TAG, "Checksum mismatch: %x (frame) != %x (calc from %s)",
                   frame_csum, calc_csum,
                   hex_repr(&bytes[0], bytes.size()).c_str());
          return false;
        }
        break;
      }
      bytes.push_back(byte);
    }
    if (bytes.size() && !reading)
      break;
    yield();
  }
  payload.assign(bytes.begin(), bytes.end());
  return true;
}

void DaikinS21Climate::write_frame(std::vector<uint8_t> payload) {
  this->tx_uart->write_byte(STX);
  this->tx_uart->write_array(payload);
  this->tx_uart->write_byte(s21_checksum(&payload[0], payload.size()));
  this->tx_uart->write_byte(ETX);
  this->tx_uart->flush();
}

bool DaikinS21Climate::s21_query(std::vector<uint8_t> code) {
  std::string c;
  for (size_t i = 0; i < code.size(); i++) {
    c += code[i];
  }
  this->write_frame(code);

  uint8_t byte;
  if (!this->rx_uart->read_byte(&byte)) {
    ESP_LOGW(TAG, "Timeout waiting for %s response", c.c_str());
    return false;
  }
  if (byte == NAK) {
    ESP_LOGD(TAG, "NAK from S21 for %s query", c.c_str());
    return false;
  }
  if (byte != ACK) {
    ESP_LOGW(TAG, "No ACK from S21 for %s query", c.c_str());
    return false;
  }

  std::vector<uint8_t> frame;
  if (!this->read_frame(frame)) {
    ESP_LOGW(TAG, "Failed reading %s response frame", c.c_str());
    return false;
  }

  this->tx_uart->write_byte(ACK);

  std::vector<uint8_t> rcode;
  std::vector<uint8_t> payload;
  for (size_t i = 0; i < frame.size(); i++) {
    if (i < code.size()) {
      rcode.push_back(frame[i]);
    } else {
      payload.push_back(frame[i]);
    }
  }

  return parse_response(rcode, payload);
}

bool DaikinS21Climate::parse_response(std::vector<uint8_t> rcode,
                                      std::vector<uint8_t> payload) {
  uint8_t mnum;  // Mode number
  bool power;    // Power state

  // ESP_LOGD(TAG, "S21: %s -> %s (%d)", str_repr(rcode).c_str(),
  //          str_repr(payload).c_str(), payload.size());

  switch (rcode[0]) {
    case 'G':      // F -> G
      switch (rcode[1]) {
        case '1':  // F1 -> Basic State
          mnum = payload[1] - '0';
          power = (payload[0] == '1');
          if (!power || mnum < 1 ||
              mnum > (sizeof(OpModes) / sizeof(OpModes[0]))) {
            this->state.mode = climate::CLIMATE_MODE_OFF;
          } else {
            this->state.mode = OpModes[mnum];
          }
          this->state.setpoint = (payload[2] - 28) * 5;  // Celsius * 10
          this->state.fan_mode = (FanMode) payload[3];
          return true;
        case '5':  // F5 -> G5 -- Swing state
          this->state.swing_v = (bool) (payload[0] & 1);
          this->state.swing_h = (bool) (payload[0] & 2);
          return true;
        case '6':  // F6 -> G6 -- Powerful
          this->state.powerful = (payload[0] == '2');
          return true;
        case '7':  // F7 -> G7 -- Econo
          this->state.econo = (payload[1] == '2');
          return true;
      }
      break;
    case 'S':      // R -> S
      switch (rcode[1]) {
        case 'H':  // Inside temperature
          this->state.inside = temp_bytes_to_c10(payload);
          return true;
        case 'I':  // Coil temperature
          this->state.coil = temp_bytes_to_c10(payload);
          return true;
        case 'a':  // Outside temperature
          this->state.outside = temp_bytes_to_c10(payload);
          return true;
        case 'L':  // Fan speed
          this->state.fan_rpm = bytes_to_num(payload) * 10;
          return true;
        case 'd':  // Compressor state / frequency? Idle if 0.
          this->state.idle =
              (payload[0] == '0' && payload[1] == '0' && payload[2] == '0');
          return true;
        default:
          if (payload.size() > 3) {
            uint16_t temp = temp_bytes_to_c10(payload);
            ESP_LOGD(TAG, "Unknown temp: %s -> %s -> %.1f C (%.1f F)",
                     str_repr(rcode).c_str(), str_repr(payload).c_str(),
                     c10_c(temp), c10_f(temp));
          }
          return false;
      }
  }
  ESP_LOGD(TAG, "Unknown response %s -> \"%s\"", str_repr(rcode).c_str(),
           str_repr(payload).c_str());
  return false;
}

void DaikinS21Climate::run_queries(std::vector<std::string> queries) {
  std::vector<uint8_t> code;

  for (auto q : queries) {
    code.clear();
    for (auto i = 0; i < q.length(); i++) {
      code.push_back(q[i]);
    }
    this->s21_query(code);
  }
}

void DaikinS21Climate::update() {
  ESP_LOGD(TAG, "** BEGIN UPDATE *****************************");
  DaikinS21State state;
  std::vector<std::string> queries = {"F1", "F5", "RH", "RI", "Ra", "RL", "Rd"};
  this->run_queries(queries);

#ifdef S21_EXPERIMENTS
  ESP_LOGD(TAG, "** UNKNOWN QUERIES **");
  // auto experiments = {"F2", "F3", "F4", "F8", "F9", "F0", "FA", "FB", "FC",
  //                     "FD", "FE", "FF", "FG", "FH", "FI", "FJ", "FK", "FL",
  //                     "FM", "FN", "FO", "FP", "FQ", "FR", "FS", "FT", "FU",
  //                     "FV", "FW", "FX", "FY", "FZ"};
  // Observed BRP device querying these.
  std::vector<std::string> experiments = {"F2", "F3", "F4", "RN",
                                          "RX", "RD", "M",  "FU0F"};
  this->run_queries(experiments);
#endif

  dump_s21_state(&this->state);
  ESP_LOGD(TAG, "** END UPDATE *****************************");
}

void DaikinS21Climate::control(const climate::ClimateCall &call) {}

}  // namespace daikin_s21
}  // namespace esphome
