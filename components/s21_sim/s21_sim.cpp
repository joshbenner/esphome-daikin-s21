#include <map>
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "s21_sim.h"

using namespace esphome;

static const char *const TAG = "s21sim";

namespace esphome {
namespace s21_sim {

#define STX 2
#define ETX 3
#define ACK 6
#define NAK 21

#define S21_RESPONSE_TIMEOUT 250

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

// void S21SIM::set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) {
//   this->uart = new UARTDevicePair();
//   this->uart->set_uart_tx_parent(tx);
//   this->uart->set_uart_rx_parent(rx);
// }

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

std::string hex_repr(std::vector<uint8_t> &bytes) {
  return hex_repr(&bytes[0], bytes.size());
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

void S21SIM::dump_config() { ESP_LOGCONFIG(TAG, "S21 Sim"); }

bool S21SIM::read_frame(std::vector<uint8_t> &payload) {
  uint8_t byte;
  std::vector<uint8_t> bytes;
  uint32_t start = millis();
  bool reading = false;
  while (true) {
    if (millis() - start > S21_RESPONSE_TIMEOUT) {
      ESP_LOGW(TAG, "Timeout waiting for frame");
      return false;
    }
    while (this->available()) {
      this->read_byte(&byte);
      if (byte == ACK) {
        ESP_LOGW(TAG, "Got ACK waiting to read start of frame");
        continue;
      } else if (!reading && byte != STX) {
        ESP_LOGW(TAG, "Unexpected byte waiting to read start of frame: 0x%02X",
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

void S21SIM::write_frame(std::vector<uint8_t> payload) {
  ESP_LOGD(TAG, "Sending: %s", str_repr(payload).c_str());
  this->write_byte(STX);
  this->write_array(payload);
  this->write_byte(s21_checksum(&payload[0], payload.size()));
  this->write_byte(ETX);
  this->flush();
}

void S21SIM::respond(std::vector<uint8_t> payload) {
  this->write_byte(ACK);
  this->write_frame(payload);
}

void S21SIM::loop() {
  if (this->available()) {
    std::vector<uint8_t> req;
    this->read_frame(req);
    ESP_LOGD(TAG, "Received req: %s", str_repr(req).c_str());
    this->handle_req(req);
  }
}

void S21SIM::handle_req(std::vector<uint8_t> req) {
  std::string code;
  for (auto i = 0; i < req.size(); i++) {
    code += req[i];
  }
  std::vector<uint8_t> res;

  if (code == "F1") {
    res.assign({'G', '1', '1', '3', 'K', 'A'});
  } else if (code == "F2") {
    res.assign({'G', '2', '=', ';', 0x00, 0x80});
  } else if (code == "F3") {
    res.assign({'G', '3', '0', 0xFE, 0xFE, 0x00});
  } else if (code == "F4") {
    res.assign({'G', '4', '0', 0x00, 0x80, 0x00});
  } else if (code == "F5") {
    res.assign({'G', '5', '0', '0', '0', 0x80});  // Not swinging
  } else if (code == "F6") {
    // nak
  } else if (code == "F7") {
    // nak
  } else if (code == "F8") {
    res.assign({'G', '8', '0', 0x00, 0x00, 0x00});
  } else if (code == "F9") {
    res.assign({'G', '9', 0xB2, 0xB4, 0xFF, 0x30});
  } else if (code == "FU0F") {  // ??
    // nak
  } else if (code == "RH") {    // Inside temp
    res.assign({'S', 'H', '0', '3', '2', '+'});
  } else if (code == "RI") {    // Coil temp ?
    res.assign({'S', 'I', '0', '9', '0', '+'});
  } else if (code == "Ra") {    // Outside temp
    res.assign({'S', 'a', '5', '1', '2', '+'});
  } else if (code == "RL") {    // Fan speed
    res.assign({'S', 'L', '0', '9', '0'});
  } else if (code == "RN") {    // ????
    res.assign({'S', 'N', '9', '5', '0', '+'});
  } else if (code == "RX") {    // ????
    res.assign({'S', 'X', '0', '4', '2', '+'});
  } else if (code == "Rd") {    // Compressor state / frequency
    res.assign({'S', 'd', '5', '1', '3'});
  } else if (code == "M") {     // ????
    res.assign({'M', '3', 'E', '5', '3'});
  } else {
    this->write_byte(NAK);
  }

  if (res.size()) {
    this->respond(res);
  } else {
    ESP_LOGE(TAG, "Unknown request: %s (%s)", str_repr(req).c_str(),
             hex_repr(req).c_str());
  }
}

}  // namespace s21_sim
}  // namespace esphome
