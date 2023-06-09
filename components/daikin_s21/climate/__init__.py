"""
Daikin S21 Mini-Split ESPHome component config validation & code generation.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor
from esphome.const import CONF_ID
from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_CLIENT_SCHEMA,
    DaikinS21Client,
)

CONF_ROOM_TEMPERATURE_SENSOR = "room_temperature_sensor"
CONF_SETPOINT_INTERVAL = "setpoint_interval"

DaikinS21Climate = daikin_s21_ns.class_(
    "DaikinS21Climate", climate.Climate, cg.PollingComponent, DaikinS21Client
)
uart_ns = cg.esphome_ns.namespace("uart")
UARTComponent = uart_ns.class_("UARTComponent")

CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(DaikinS21Climate),
            cv.Optional(CONF_ROOM_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(
                CONF_SETPOINT_INTERVAL, default="300s"
            ): cv.positive_time_period_seconds,
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(S21_CLIENT_SCHEMA)
)


async def to_code(config):
    """Generate code"""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    s21_var = await cg.get_variable(config[CONF_S21_ID])
    cg.add(var.set_s21(s21_var))
    if CONF_ROOM_TEMPERATURE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_ROOM_TEMPERATURE_SENSOR])
        cg.add(var.set_room_sensor(sens))
        if CONF_SETPOINT_INTERVAL in config:
            cg.add(var.set_setpoint_interval(config[CONF_SETPOINT_INTERVAL]))
