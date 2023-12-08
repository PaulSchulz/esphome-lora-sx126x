import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@PaulSchulz"]
AUTO_LOAD = [ "sensor","text_sensor"]
DEPENDANCIES = ["spi"]

lorawan_sx126x_ns = cg.esphome_ns.namespace("lorawan_sx126x")

# empty_component_ns = cg.esphome_ns.namespace('empty_component')
LoRaWANSX126X = lorawan_sx126x_ns.class_('LoRaWANSX126X', cg.Component)

# Hardware
# Heltec Wifi LoRa 32 (V3) - SX126x pin configuration
PIN_LORA_RESET = 12  # LORA RESET
PIN_LORA_DIO_1 = 14  # LORA DIO_1
PIN_LORA_BUSY  = 13  # LORA SPI BUSY
PIN_LORA_NSS   =  8  # LORA SPI CS
PIN_LORA_SCLK  =  9  # LORA SPI CLK
PIN_LORA_MISO  = 11  # LORA SPI MISO
PIN_LORA_MOSI  = 10  # LORA SPI MOSI
RADIO_TXEN     = -1  # LORA ANTENNA TX ENABLE
RADIO_RXEN     = -1  # LORA ANTENNA RX ENABLE

# LoRaWAN Radio Parameters
CONF_LORAWAN_DEVICE_EUI        = ""

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LoraSX126X),
    cv.Optional('name'): cv.string,

    cv.Optional('pin_lora_reset', default=PIN_LORA_RESET): cv.int_,
    cv.Optional('pin_lora_dio_1', default=PIN_LORA_DIO_1): cv.int_,
    cv.Optional('pin_lora_busy',  default=PIN_LORA_BUSY):  cv.int_,
    cv.Optional('pin_lora_nss',   default=PIN_LORA_NSS):   cv.int_,
    cv.Optional('pin_lora_sclk',  default=PIN_LORA_SCLK):  cv.int_,
    cv.Optional('pin_lora_miso',  default=PIN_LORA_MISO):  cv.int_,
    cv.Optional('pin_lora_mosi',  default=PIN_LORA_MOSI):  cv.int_,
    cv.Optional('radio_txen',     default=RADIO_TXEN):     cv.int_,
    cv.Optional('radio_rxen',     default=RADIO_RXEN):     cv.int_,

    cv.Required('region'):        cv.string,
    cv.Required('authtype'):      cv.string,
    cv.Optional('device_eui',     default=CONF_LORAWAN_DEVICE_EUI):         cv.string,
    cv.Required('app_eui'):       cv.string,
    cv.Required('app_key'):       cv.string,

}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)

    cg.add(var.set_pin_lora_reset(config['pin_lora_reset']))
    cg.add(var.set_pin_lora_dio_1(config['pin_lora_dio_1']))
    cg.add(var.set_pin_lora_busy(config['pin_lora_busy']))
    cg.add(var.set_pin_lora_nss(config['pin_lora_nss']))
    cg.add(var.set_pin_lora_sclk(config['pin_lora_sclk']))
    cg.add(var.set_pin_lora_miso(config['pin_lora_miso']))
    cg.add(var.set_pin_lora_mosi(config['pin_lora_mosi']))
    cg.add(var.set_radio_txen(config['radio_txen']))
    cg.add(var.set_radio_rxen(config['radio_rxen']))

    cg.add(var.set_lorawan_region(config['region']))
    cg.add(var.set_lorawan_authtype(config['authtype']))
    cg.add(var.set_lorawan_device_eui(config['device_eui']))
    cg.add(var.set_lorawan_app_eui(config['app_eui']))
    cg.add(var.set_lorawan_app_key(config['app_key']))
