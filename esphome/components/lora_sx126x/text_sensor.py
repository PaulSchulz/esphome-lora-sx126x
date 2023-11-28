import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

#from esphome.const import (
#    STATE_CLASS_MEASUREMENT,
#)

lora_sx126x_rssi_ns = cg.esphome_ns.namespace("lora_sx126x")
#                                              ^^^^^^^^^^^
#                                                   ^
#                                                   |
#  C++ namespace under "esphome::"" ----------------'

PktSensor = lora_sx126x_rssi_ns.class_(
    "LoraSX126Xpkt", text_sensor.TextSensor, cg.Component
)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(
        PktSensor,
    )
)

async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)
