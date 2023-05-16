#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace s21_sim {

class S21SIM : public Component, public uart::UARTDevice {
 public:
  void loop() override;
  void dump_config() override;
  // void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx);

  bool read_frame(std::vector<uint8_t> &payload);
  void write_frame(std::vector<uint8_t> payload);
  void respond(std::vector<uint8_t> payload);

  void handle_req(std::vector<uint8_t> req);

 protected:
  // UARTDevicePair *uart;
};

}  // namespace s21_sim
}  // namespace esphome
