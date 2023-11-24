import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from esphome.const import (
    STATE_CLASS_MEASUREMENT,
)

lora_sx126x_rssi_ns = cg.esphome_ns.namespace("lora_sx126x")
#                                              ^^^^^^^^^^^
#                                                   ^
#                                                   |
#  C++ namespace under "esphome::"" ----------------'

RSSISensor = lora_sx126x_rssi_ns.class_(
    "LoraSX126Xrssi", sensor.Sensor, cg.Component
    # ^^^^^^^^^   ^^^^^^^^^^^^^
    #     ^             ^
    #     |             |
    #     |             `-- Inherenting from Sensor component
    #     `----- Class name of object
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        RSSISensor,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    )
)

async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
