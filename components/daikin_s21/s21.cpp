#include <cinttypes>
#include <numeric>
#include "s21.h"
#include <sys/types.h>

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

#define STX 2
#define ETX 3
#define ENQ 5
#define ACK 6
#define NAK 21

static const char *const TAG = "daikin_s21";

std::string daikin_climate_mode_to_string(DaikinClimateMode mode) {
  switch (mode) {
    case DaikinClimateMode::Disabled:
      return "Disabled";
    case DaikinClimateMode::Auto:
      return "Auto";
    case DaikinClimateMode::Dry:
      return "Dry";
    case DaikinClimateMode::Cool:
      return "Cool";
    case DaikinClimateMode::Heat:
      return "Heat";
    case DaikinClimateMode::Fan:
      return "Fan";
    default:
      return "UNKNOWN";
  }
}

std::string daikin_fan_mode_to_string(DaikinFanMode mode) {
  switch (mode) {
    case DaikinFanMode::Auto:
      return "Auto";
    case DaikinFanMode::Silent:
      return "Silent";
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
    default:
      return "UNKNOWN";
  }
}

int16_t bytes_to_num(uint8_t *bytes, size_t len) {
  // <ones><tens><hundreds><neg/pos>
  int16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if (len > 3 && bytes[3] == '-')
    val *= -1;
  return val;
}

int16_t temp_bytes_to_c10(uint8_t *bytes) { return bytes_to_num(bytes, 4); }

int16_t temp_f9_byte_to_c10(uint8_t *bytes) { return (*bytes / 2 - 64) * 10; }

uint8_t c10_to_setpoint_byte(int16_t setpoint) {
  return (setpoint + 3) / 5 + 28;
}

void DaikinS21::set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) {
  this->tx_uart = tx;
  this->rx_uart = rx;
}

#define S21_BAUD_RATE 2400
#define S21_STOP_BITS 2
#define S21_DATA_BITS 8
#define S21_PARITY uart::UART_CONFIG_PARITY_EVEN

void DaikinS21::check_uart_settings() {
  for (auto uart : {this->tx_uart, this->rx_uart}) {
    if (uart->get_baud_rate() != S21_BAUD_RATE) {
      ESP_LOGE(
          TAG,
          "  Invalid baud_rate: Integration requested baud_rate %u but you "
          "have %" PRIu32 "!",
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

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32, this->get_update_interval());
  this->check_uart_settings();
}

// Adapated from ESPHome UART debugger
std::string hex_repr(const uint8_t *bytes, size_t len) {
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
std::string str_repr(const uint8_t *bytes, size_t len) {
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

void DaikinS21::write_frame(const uint8_t *payload, size_t payload_len) {
  size_t command_len = strlen(tx_command);
  if (command_len > S21_MAX_COMMAND_SIZE) {
    ESP_LOGE(TAG, "Tx: Command '%s' too large", tx_command);
    comm_state = CommState::Error;
    return;
  }

  if (this->debug_protocol) {
    ESP_LOGV(TAG, "Tx: %s %s %s", tx_command,
             str_repr(payload, payload_len).c_str(),
             hex_repr(payload, payload_len).c_str());
  }

  this->tx_uart->write_byte(STX);
  this->tx_uart->write_str(tx_command);
  uint8_t checksum = std::accumulate(tx_command, tx_command + command_len, 0U);
  if (payload_len) {
    this->tx_uart->write_array(payload, payload_len);
    checksum = std::accumulate(payload, payload + payload_len, checksum);
  }
  if (checksum == STX) {
    checksum = ENQ;  // escape mid-message STX characters
  }
  this->tx_uart->write_byte(checksum);
  this->tx_uart->write_byte(ETX);
}

void DaikinS21::tx_next_command() {
  uint8_t payload[S21_MAX_PAYLOAD_SIZE];

  std::fill(std::begin(payload), std::end(payload),
            static_cast<uint8_t>('0'));  // prefill to save lines later

  // select next command / query
  if (activate_climate) {
    activate_climate = false;
    tx_command = "D1";
    payload[0] = static_cast<uint8_t>(pending.power_on ? '1' : '0');
    payload[1] = static_cast<uint8_t>(pending.mode);
    payload[2] = static_cast<uint8_t>(
        c10_to_setpoint_byte(lroundf(round(pending.setpoint * 2) / 2 * 10.0)));
    payload[3] = static_cast<uint8_t>(pending.fan);
    comm_state = CommState::CommandAck;
  }
  else if (activate_swing_mode) {
    activate_swing_mode = false;
    tx_command = "D5";  // todo encoding deviates from faikin
    payload[0] = static_cast<uint8_t>(
        '0' + (pending.swing_h && pending.swing_v ? 4 : 0) +
        (pending.swing_h ? 2 : 0) + (pending.swing_v ? 1 : 0));
    payload[1] =
        static_cast<uint8_t>(pending.swing_v || pending.swing_h ? '?' : '0');
    comm_state = CommState::CommandAck;
  }
  else if (current_query != queries.end()) {
    // query scan underway, continue
    tx_command = *current_query;
    comm_state = CommState::QueryAck;
  }
  else if (refresh_state) {
    // start fresh query scan
    refresh_state = false;
    current_query = queries.begin();
    tx_command = *current_query;
    comm_state = CommState::QueryAck;
  }
  else {
    return;  // nothing to do
  }

  // transmit and prep for response
  if (comm_state == CommState::CommandAck) {
    write_frame(payload, 4);
  } else {
    write_frame(nullptr, 0);
  }
  rx_timeout = millis();
  rx_buffer.clear();
}

void DaikinS21::parse_command_response() {
  char rcode[S21_MAX_COMMAND_SIZE + 1] = {};
  uint8_t payload[S21_MAX_PAYLOAD_SIZE];
  const size_t rcode_len = strlen(tx_command);
  const size_t payload_len = rx_buffer.size() - rcode_len;

  std::copy_n(&rx_buffer[0], rcode_len, rcode);
  std::copy_n(&rx_buffer[rcode_len], payload_len, payload);

  switch (rcode[0]) {
    case 'G':  // F -> G
      switch (rcode[1]) {
        case '1':  // F1 -> Basic State
          this->active.power_on = (payload[0] == '1');
          this->active.mode = static_cast<DaikinClimateMode>(payload[1]);
          this->active.setpoint = ((payload[2] - 28) * 5);  // Celsius * 10
          if (this->support_rg == false) {  // prefer RG (silent mode not reported here)
            this->active.fan = static_cast<DaikinFanMode>(payload[3]);
          }
          this->ready.set(ReadyBasic);
          return;
        case '5':  // F5 -> G5 -- Swing state
          this->active.swing_v = payload[0] & 1;
          this->active.swing_h = payload[0] & 2;
          this->ready.set(ReadySwing);
          break;
        case '8':  // F8 -> G8 -- Protocol version. Always 0 for me.
          break;
        case '9':  // F9 -> G9 -- Inside temperature, todo don't poll if unit supports RH and Ra (0.5 degree granularity there)
          this->temp_inside = temp_f9_byte_to_c10(&payload[0]);
          this->temp_outside = temp_f9_byte_to_c10(&payload[1]);
          break;
        default:
          break;
      }
      break;

    case 'S':  // R -> S
      switch (rcode[1]) {
        case 'B':  // Operational mode, single character, same info as G1
          return;
        case 'G':  // Fan mode, better detail than G1
          this->active.fan = static_cast<daikin_s21::DaikinFanMode>(payload[0]);
          this->support_rg = true;
          return;
        case 'H':  // Inside temperature
          this->temp_inside = temp_bytes_to_c10(payload);
          return;
        case 'I':  // Coil temperature
          this->temp_coil = temp_bytes_to_c10(payload);
          return;
        case 'a':  // Outside temperature
          this->temp_outside = temp_bytes_to_c10(payload);
          return;
        case 'L':  // Fan speed
          this->fan_rpm = bytes_to_num(payload, payload_len) * 10;
          return;
        case 'd':  // Compressor frequency in hertz, idle if 0.
          this->compressor_hz = bytes_to_num(payload, payload_len);
          this->ready.set(ReadyCompressor);
          return;
        case 'C':  // Setpoint, same info as G1
          this->active.setpoint = bytes_to_num(payload, payload_len);
          return;
        case 'N':
          this->swing_vertical_angle = bytes_to_num(payload, 4);
          return;
        case 'F':  // Swing mode, same info as G5. ascii hex string: 2F herizontal 1F vertical 7F both 00 off
          break;
        case 'M':  // related to v_swing somehow
        case 'X':
        case 'z':
        default:
          if (payload_len > 3) {
            int8_t temp = temp_bytes_to_c10(payload);
            ESP_LOGD(TAG, "Unknown sensor: %s -> %s -> %.1f C (%.1f F)", rcode,
                     hex_repr(payload, payload_len).c_str(), c10_c(temp),
                     c10_f(temp));
          }
          break;
      }

    case 'M':  // todo faikin says 100WH units of power
      break;

    default:
      break;
  }

  // protocol decoding debug
  // note: well known responses return directly from the switch statements
  // break instead if you want to view their contents

  // print everything
  // if (this->debug_protocol) {
  //   ESP_LOGD(TAG, "S21: %s -> %s %s", rcode,
  //            str_repr(payload, payload_len).c_str(),
  //            hex_repr(payload, payload_len).c_str());
  // }

  // print changed values
  if (this->debug_protocol) {
    auto curr = std::vector<uint8_t>(payload, payload + payload_len);
    if (val_cache[rcode] != curr) {
      const auto prev = val_cache[rcode];
      ESP_LOGI(TAG, "S21 %s changed: %s %s -> %s %s", rcode,
               str_repr(prev.data(), prev.size()).c_str(),
               hex_repr(prev.data(), prev.size()).c_str(),
               str_repr(curr.data(), curr.size()).c_str(),
               hex_repr(curr.data(), curr.size()).c_str());
      val_cache[rcode] = curr;
    }
  }
}

void DaikinS21::handle_rx_byte(uint8_t byte) {
  const char *const tx_type =
      (comm_state == CommState::CommandAck) ? "command" : "query";

  switch (comm_state) {
    case CommState::QueryAck:
    case CommState::CommandAck:
      switch (byte) {
        case ACK:
          if (comm_state == CommState::QueryAck) {
            comm_state = CommState::QueryStx;
          } else {
            ESP_LOGV(TAG, "Rx: ACK from S21 for command %s", tx_command);
            comm_state = CommState::Idle;
            rx_timeout = millis();
            refresh_state = true;  // command took, refresh whole state
          }
          break;
        case NAK:
          ESP_LOGW(TAG, "Rx: NAK from S21 for %s %s", tx_type, tx_command);
          if (comm_state == CommState::QueryAck) {
            ESP_LOGW(TAG, "Removing %s from query pool (assuming unsupported)",
                     tx_command);
            queries.erase(current_query);
          }
          comm_state = CommState::Idle;
          rx_timeout = millis();
          break;
        default:
          ESP_LOGW(TAG, "Rx: No ACK from S21 for %s %s", tx_type, tx_command);
          comm_state = CommState::Error;
          break;
      }
      break;

    case CommState::QueryStx:
      if (byte == STX) {
        comm_state = CommState::QueryEtx;
        rx_buffer = {};  // prepare for payload
      } else {
        ESP_LOGW(TAG, "Rx: No STX from S21 for query %s", tx_command);
        comm_state = CommState::Error;
      }
      break;

    case CommState::QueryEtx:
      if (byte != ETX) {
        // not the end, add to buffer
        rx_buffer.push_back(byte);
        if (rx_buffer.size() >
            (S21_MAX_COMMAND_SIZE + S21_MAX_PAYLOAD_SIZE + 1)) {
          ESP_LOGW(TAG, "Rx: Overflow from query %s response", tx_command);
          comm_state = CommState::Error;
        }
      } else {
        // frame received, validate
        byte = rx_buffer[rx_buffer.size() - 1];
        rx_buffer.pop_back();
        {
          uint8_t calc_csum =
              std::accumulate(rx_buffer.begin(), rx_buffer.end(), 0U);
          if ((calc_csum == byte) ||
              ((calc_csum == STX) &&
               (byte == ENQ))) {  // protocol avoids STX in message body
            tx_uart->write_byte(ACK);
            parse_command_response();
            current_query++;
            comm_state = CommState::Idle;
            rx_timeout = millis();
          } else {
            ESP_LOGW(TAG,
                     "Rx: Checksum mismatch: %x (frame) != %x (calc from %s)",
                     byte, calc_csum,
                     hex_repr(rx_buffer.data(), rx_buffer.size()).c_str());
            comm_state = CommState::Error;
          }
        }
      }
      break;
    default:
      break;
  }
}

void DaikinS21::setup() {
  // populate messages to poll
  // clang-format off
#define S21_EXPERIMENTS 1
  queries = {
      "F1", "F5", "F9",
      "Rd", "RH", "RI", "Ra", "RL", "RG",
      // redundant/worse: "RC", "RF", "RB",
#if S21_EXPERIMENTS
      // Observed BRP device querying these.
      // "F2", "F3", "F4", "RN",
      // "RX", "RD", "M", "FU0F",
      // Query Experiments
      // "RA", 
      // "RE",
      // "RK", "RM", "RW",
      // "Rb", "Re", "Rg", "Rz",
#endif
  };
  // clang-format on

  current_query = queries.begin();
}

void DaikinS21::loop() {
  switch (comm_state) {
    case CommState::Idle:
      if ((millis() - rx_timeout) > S21_RESPONSE_TURNAROUND) {
        tx_next_command();
      }
      break;

    case CommState::Error:
      if ((millis() - rx_timeout) > S21_ERROR_TIMEOUT) {
        ESP_LOGD(TAG, "Error state expired");
        comm_state = CommState::Idle;
        rx_timeout = millis();
        rx_uart->flush();
        current_query = queries.end();
        refresh_state = true;
        activate_climate = false;
        activate_swing_mode = false;
      }
      break;

    default:  // all other states are actively receiving data from the unit
      // timed out?
      if ((millis() - rx_timeout) > S21_RESPONSE_TIMEOUT) {
        ESP_LOGW(TAG, "Timeout waiting for %s response", tx_command);
        comm_state = CommState::Idle;
        break;
      }
      // read all available bytes
      while (rx_uart->available()) {
        uint8_t byte;
        rx_uart->read_byte(&byte);
        handle_rx_byte(byte);
        rx_timeout = millis();
      }
      break;
  }
}

void DaikinS21::update() {
  refresh_state = true;

  static bool ready_printed = false;
  if (!ready_printed && this->is_ready()) {
    ESP_LOGI(TAG, "Daikin S21 Ready");
    ready_printed = true;
  }

  if (this->debug_protocol) {
    this->dump_state();
  }
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "** BEGIN STATE *****************************");

  ESP_LOGD(TAG, "  Power: %s", ONOFF(this->active.power_on));
  ESP_LOGD(TAG, "   Mode: %s (%s)",
           daikin_climate_mode_to_string(this->active.mode).c_str(),
           this->is_idle() ? "idle" : "active");
  float degc = this->active.setpoint / 10.0;
  float degf = degc * 1.8 + 32.0;
  ESP_LOGD(TAG, " Target: %.1f C (%.1f F)", degc, degf);
  ESP_LOGD(TAG, "    Fan: %s (%d rpm)",
           daikin_fan_mode_to_string(this->active.fan).c_str(), this->fan_rpm);
  ESP_LOGD(TAG, "  Swing: H:%s V:%s", YESNO(this->active.swing_h),
           YESNO(this->active.swing_h));
  ESP_LOGD(TAG, " Inside: %.1f C (%.1f F)", c10_c(this->temp_inside),
           c10_f(this->temp_inside));
  ESP_LOGD(TAG, "Outside: %.1f C (%.1f F)", c10_c(this->temp_outside),
           c10_f(this->temp_outside));
  ESP_LOGD(TAG, "   Coil: %.1f C (%.1f F)", c10_c(this->temp_coil),
           c10_f(this->temp_coil));

  ESP_LOGD(TAG, "** END STATE *****************************");
}

void DaikinS21::set_daikin_climate_settings(bool power_on,
                                            DaikinClimateMode mode,
                                            float setpoint,
                                            DaikinFanMode fan_mode) {
  pending.power_on = power_on;
  pending.mode = mode;
  pending.setpoint = setpoint;
  pending.fan = fan_mode;
  activate_climate = true;
}

void DaikinS21::set_swing_settings(const bool swing_v, const bool swing_h) {
  pending.swing_v = swing_v;
  pending.swing_h = swing_h;
  activate_swing_mode = true;
}

}  // namespace daikin_s21
}  // namespace esphome
