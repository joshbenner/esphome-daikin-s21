#include "s21.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

#define STX 2
#define ETX 3
#define ACK 6
#define NAK 21

#define S21_RESPONSE_TIMEOUT 250

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

int16_t bytes_to_num(std::vector<uint8_t> &bytes) {
  return bytes_to_num(&bytes[0], bytes.size());
}

int16_t temp_bytes_to_c10(uint8_t *bytes) { return bytes_to_num(bytes, 4); }

int16_t temp_bytes_to_c10(std::vector<uint8_t> &bytes) {
  return temp_bytes_to_c10(&bytes[0]);
}

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

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
  this->check_uart_settings();
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

bool DaikinS21::read_frame(std::vector<uint8_t> &payload) {
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
          // This sometimes happens with G9 reply, no idea why
          if (bytes[0] == 0x47 && bytes[1] == 0x39) {
            calc_csum += 2;
          }
          if (calc_csum != frame_csum) {
            ESP_LOGW(TAG, "Checksum mismatch: %x (frame) != %x (calc from %s)",
            frame_csum, calc_csum,
            hex_repr(&bytes[0], bytes.size()).c_str());
            return false;
          }
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

void DaikinS21::write_frame(std::vector<uint8_t> frame) {
  this->tx_uart->write_byte(STX);
  this->tx_uart->write_array(frame);
  this->tx_uart->write_byte(s21_checksum(&frame[0], frame.size()));
  this->tx_uart->write_byte(ETX);
  this->tx_uart->flush();
}

bool DaikinS21::s21_query(std::vector<uint8_t> code) {
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

bool DaikinS21::parse_response(std::vector<uint8_t> rcode,
                               std::vector<uint8_t> payload) {
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "S21: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
  }

  switch (rcode[0]) {
    case 'G':      // F -> G
      switch (rcode[1]) {
        case '1':  // F1 -> Basic State
          this->power_on = (payload[0] == '1');
          this->mode = (DaikinClimateMode) payload[1];
          this->setpoint = ((payload[2] - 28) * 5);  // Celsius * 10
          this->fan = (DaikinFanMode) payload[3];
          return true;
        case '5':  // F5 -> G5 -- Swing state
          this->swing_v = payload[0] & 1;
          this->swing_h = payload[0] & 2;
          return true;
        case '9':  // F9 -> G9 -- Inside temperature
          this->temp_inside = temp_f9_byte_to_c10(&payload[0]);
          this->temp_outside = temp_f9_byte_to_c10(&payload[1]);
          return true;
      }
      break;
    case 'S':      // R -> S
      switch (rcode[1]) {
        case 'H':  // Inside temperature
          this->temp_inside = temp_bytes_to_c10(payload);
          return true;
        case 'I':  // Coil temperature
          this->temp_coil = temp_bytes_to_c10(payload);
          return true;
        case 'a':  // Outside temperature
          this->temp_outside = temp_bytes_to_c10(payload);
          return true;
        case 'L':  // Fan speed
          this->fan_rpm = bytes_to_num(payload) * 10;
          return true;
        case 'd':  // Compressor state / frequency? Idle if 0.
          this->idle =
              (payload[0] == '0' && payload[1] == '0' && payload[2] == '0');
          return true;
        default:
          if (payload.size() > 3) {
            int8_t temp = temp_bytes_to_c10(payload);
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

bool DaikinS21::run_queries(std::vector<std::string> queries) {
  bool success = true;

  for (auto q : queries) {
    std::vector<uint8_t> code(q.begin(), q.end());
    success = this->s21_query(code) && success;
  }

  return success;  // True if all queries successful
}

void DaikinS21::update() {
  std::vector<std::string> queries = {"F1", "F5", "Rd"};
  // These queries might fail but they won't affect the basic functionality
  std::vector<std::string> failable_queries = {"F9", "RH", "RI", "Ra", "RL"};
  if (this->run_queries(queries)) {
    this->run_queries(failable_queries);
    if(!this->ready) {
      ESP_LOGI(TAG, "Daikin S21 Ready");
      this->ready = true;
    }
  }
  if (this->debug_protocol) {
    this->dump_state();
  }

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
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "** BEGIN STATE *****************************");

  ESP_LOGD(TAG, "  Power: %s", ONOFF(this->power_on));
  ESP_LOGD(TAG, "   Mode: %s (%s)",
           daikin_climate_mode_to_string(this->mode).c_str(),
           this->idle ? "idle" : "active");
  float degc = this->setpoint / 10.0;
  float degf = degc * 1.8 + 32.0;
  ESP_LOGD(TAG, " Target: %.1f C (%.1f F)", degc, degf);
  ESP_LOGD(TAG, "    Fan: %s (%d rpm)",
           daikin_fan_mode_to_string(this->fan).c_str(), this->fan_rpm);
  ESP_LOGD(TAG, "  Swing: H:%s V:%s", YESNO(this->swing_h),
           YESNO(this->swing_h));
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
  // clang-format off
  std::vector<uint8_t> cmd = {
    (uint8_t)(power_on ? '1' : '0'),
    (uint8_t) mode,
    c10_to_setpoint_byte(lroundf(round(setpoint * 2) / 2 * 10.0)),
    (uint8_t) fan_mode
  };
  // clang-format on
  ESP_LOGD(TAG, "Sending basic climate CMD (D1): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '1'}, cmd)) {
    ESP_LOGW(TAG, "Failed basic climate CMD");
  } else {
    this->update();
  }
}

void DaikinS21::set_swing_settings(bool swing_v, bool swing_h) {
  std::vector<uint8_t> cmd = {
      (uint8_t) ('0' + (swing_h ? 2 : 0) + (swing_v ? 1 : 0) +
                 (swing_h && swing_v ? 4 : 0)),
      (uint8_t) (swing_v || swing_h ? '?' : '0'), '0', '0'};
  ESP_LOGD(TAG, "Sending swing CMD (D5): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '5'}, cmd)) {
    ESP_LOGW(TAG, "Failed swing CMD");
  } else {
    this->update();
  }
}

bool DaikinS21::send_cmd(std::vector<uint8_t> code,
                         std::vector<uint8_t> payload) {
  std::vector<uint8_t> frame;
  uint8_t byte;

  for (auto b : code) {
    frame.push_back(b);
  }
  for (auto b : payload) {
    frame.push_back(b);
  }

  this->write_frame(frame);
  if (!this->rx_uart->read_byte(&byte)) {
    ESP_LOGW(TAG, "Timeout waiting for ACK to %s", str_repr(frame).c_str());
    return false;
  }
  if (byte == NAK) {
    ESP_LOGW(TAG, "Got NAK for frame: %s", str_repr(frame).c_str());
    return false;
  }
  if (byte != ACK) {
    ESP_LOGW(TAG, "Unexpected byte waiting for ACK: %s",
             str_repr(&byte, 1).c_str());
    return false;
  }

  return true;
}

}  // namespace daikin_s21
}  // namespace esphome
