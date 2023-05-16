"""
Pretend to be a Daikin mini split.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

CONF_TX_UART = "tx_uart"
CONF_RX_UART = "rx_uart"

s21_sim_ns = cg.esphome_ns.namespace("s21_sim")
S21SIM = s21_sim_ns.class_("S21SIM", cg.Component, uart.UARTDevice)
uart_ns = cg.esphome_ns.namespace("uart")
UARTComponent = uart_ns.class_("UARTComponent")

CONFIG_SCHEMA = (cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(S21SIM),
            # cv.Required(CONF_TX_UART): cv.use_id(UARTComponent),
            # cv.Required(CONF_RX_UART): cv.use_id(UARTComponent),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    """Generate code"""
    sim = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(sim, config)
    # tx_uart = await cg.get_variable(config[CONF_TX_UART])
    # rx_uart = await cg.get_variable(config[CONF_RX_UART])
    # cg.add(sim.set_uarts(tx_uart, rx_uart))
    await uart.register_uart_device(sim, config)
