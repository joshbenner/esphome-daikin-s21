#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/components/uart/uart_component.h"

namespace esphome {
namespace daikin_s21 {

using namespace uart;

class UARTDevicePair : public UARTDevice {
 public:
  void set_uart_tx_parent(UARTComponent *parent) {
    this->set_uart_parent(parent);
  }
  void set_uart_rx_parent(UARTComponent *parent) { this->rx_parent_ = parent; }

  int available() { return this->rx_parent_->available(); }
  bool read_byte(uint8_t *data) { return this->rx_parent_->read_byte(data); }
  bool peek_byte(uint8_t *data) { return this->rx_parent_->peek_byte(data); }
  bool read_array(uint8_t *data, size_t len) {
    return this->rx_parent_->read_array(data, len);
  }

 protected:
  UARTComponent *rx_parent_{nullptr};
};

}  // namespace daikin_s21
}  // namespace esphome
