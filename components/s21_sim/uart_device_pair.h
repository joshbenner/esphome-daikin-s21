// #pragma once

// #include "esphome/components/uart/uart.h"
// #include "esphome/components/uart/uart_component.h"

// namespace esphome {
// namespace s21_sim {

// using namespace uart;

// #define S21_RESPONSE_TIMEOUT 250

// class UARTDevicePair : public UARTDevice {
//  public:
//   void set_uart_tx_parent(UARTComponent *parent) {
//     this->set_uart_parent(parent);
//   }
//   void set_uart_rx_parent(UARTComponent *parent) { this->rx_parent_ = parent; }

//   int available() { return this->rx_parent_->available(); }
//   // bool read_byte(uint8_t *data) { return this->rx_parent_->read_byte(data); }
//   bool peek_byte(uint8_t *data) { return this->rx_parent_->peek_byte(data); }
//   bool read_array(uint8_t *data, size_t len) {
//     return this->rx_parent_->read_array(data, len);
//   }
//   void flush_rx() {
//     uint8_t byte;
//     while (this->rx_parent_->available() > 0) {
//       this->rx_parent_->read_byte(&byte);
//     }
//   }

//   bool read_bytes(uint8_t *bytes, size_t len, uint32_t timeout) {
//     uint32_t start = millis();
//     while (this->rx_parent_->available() < len) {
//       if (millis() - start > timeout) {
//         ESP_LOGD(TAG, "Timeout waiting for byte %u",
//                  this->rx_parent_->available());
//         return false;
//       }
//       yield();
//     }
//     this->rx_parent_->read_array(bytes, len);
//     return true;
//   }

//   bool read_bytes(uint8_t *bytes, size_t len) {
//     return this->read_bytes(bytes, len, S21_RESPONSE_TIMEOUT);
//   }

//   bool read_byte(uint8_t *byte, uint32_t timeout) {
//     return this->read_bytes(byte, 1, timeout);
//   }

//   bool read_byte(uint8_t *byte) {
//     return this->read_byte(byte, S21_RESPONSE_TIMEOUT);
//   }

//   void write_bytes(uint8_t *bytes, size_t len) {
//     this->parent_->write_array(bytes, len);
//   }

//   void write_bytes(std::vector<uint8_t> &bytes) {
//     this->parent_->write_array(bytes);
//   }

//   void write_byte(uint8_t byte) { this->parent_->write_byte(byte); }

//  protected:
//   UARTComponent *rx_parent_{nullptr};
// };

// }  // namespace s21_sim
// }  // namespace esphome
