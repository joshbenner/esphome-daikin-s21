# esphome-daikin-s21

ESPHome component to control Daikin indoor mini-split units with s21 ports.

## Example

```yaml
logger:
  baud_rate: 0  # Disable UART logger if using UART0 (pins 1,3)

uart:
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 2400

climate:
  - platform: daikin_s21
    name: My Daikin Mini-Split
```

## Daikin S21 UART Notes

* Baud rate: 2400
* Data bits: 8
* Parity: Even
* Stop bits: 2
* Flow Ctrl: No
