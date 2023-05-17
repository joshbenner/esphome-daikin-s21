"""
Daikin S21 Mini-Split ESPHome component config validation & code generation.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

CONF_TX_UART = "tx_uart"
CONF_RX_UART = "rx_uart"

daikin_s21_ns = cg.esphome_ns.namespace("daikin_s21")
DaikinS21 = daikin_s21_ns.class_("DaikinS21", cg.PollingComponent)
uart_ns = cg.esphome_ns.namespace("uart")
UARTComponent = uart_ns.class_("UARTComponent")

CONFIG_SCHEMA = (cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DaikinS21),
            cv.Required(CONF_TX_UART): cv.use_id(UARTComponent),
            cv.Required(CONF_RX_UART): cv.use_id(UARTComponent),
        }
    )
    .extend(cv.polling_component_schema("5s"))
)


async def to_code(config):
    """Generate code"""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    tx_uart = await cg.get_variable(config[CONF_TX_UART])
    rx_uart = await cg.get_variable(config[CONF_RX_UART])
    cg.add(var.set_uarts(tx_uart, rx_uart))
