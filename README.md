# esphome-daikin-s21

ESPHome component to control Daikin indoor mini-split units with s21 ports.

Many thanks to the work by [revk][1] on the fantastic [Faikin][2] project, which
was the primary inspiration and guide for building this ESPHome component.

## Features

- Setpoint temperature.
- Climate modes COOL, HEAT, DRY, FAN, and HEAT_COOL.
- Fan modes auto and 1-5.
- Swing modes horizontal, vertical, and both.

Sensors:
* Inside temperature (usually measured at indoor air handler return)
* Outside temperature (outside exchanger)
* Coil temperature (indoor air handler's coil)
* Fan speed

## Limitations

* This code has only been tested on ESP32 pico.
* Does not detect nor support powerful, econo, quiet modes.
* Does not support comfort or presence detection features on some models.
* Does not interact with the indoor units schedules (do that with HA instead).

## Hardware

### S21 Port

**NOTE:** The Daikin S21 port provides >5V, so if you intend to power your
board on this pin, be sure to test its output and regulate voltage accordingly.

On my Daikin units, the S21 port has the following pins:

* 1 - Unused
* 2 - TX (5V)
* 3 - RX (5V)
* 4 - VCC (>5V!!)
* 5 - GND

The S21 plug is JST `EHR-5` and related header `B5B-EH-A(LF)(SN)`, though the
plug pins are at standard pin header widths.

### PCB

I've been using the board designed by [revk][1] available [here][3]. Note that
revk's design includes a FET that inverts the logic levels on the ESP's RX pin,
which required using two separate UART devices to get around an ESPHome limit
on having pins inverted differently.

[1]: https://github.com/revk
[2]: https://github.com/revk/ESP32-Faikin
[3]: https://github.com/revk/ESP32-Faikin/tree/main/PCB/Faikin

## Configuration Example

```yaml
logger:
  baud_rate: 0  # Disable UART logger if using UART0 (pins 1,3)

external_components:
  - source: github://joshbenner/esphome-daikin-s21@main
    components: [ daikin_s21 ]

uart:
  - id: s21_uart
    tx_pin: GPIO1
    rx_pin: GPIO3
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 2

# The base UART communication hub.
daikin_s21:
  tx_uart: s21_uart
  rx_uart: s21_uart

climate:
  - name: My Daikin
    platform: daikin_s21
    visual:
      temperature_step: 1.0

# Optional additional sensors.
sensor:
  - platform: daikin_s21
    inside_temperature:
      name: My Daikin Inside Temperature
    outside_temperature:
      name: My Daikin Outside Temperature
    coil_temperature:
      name: My Daikin Coil Temperature
    fan_speed:
      name: My Daikin Fan Speed
```

Here is an example of how daikin_s21 can be used with one inverted UART pin:

```yaml
uart:
  - id: s21_tx
    tx_pin: 26
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 2

  - id: s21_rx
    rx_pin:
      number: 27
      inverted: true
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 2

daikin_s21:
  tx_uart: s21_tx
  rx_uart: s21_rx
```
