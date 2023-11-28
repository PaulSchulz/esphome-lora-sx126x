import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@PaulSchulz"]
AUTO_LOAD = [ "sensor","text_sensor"]
DEPENDANCIES = ["spi"]

lora_sx126x_ns = cg.esphome_ns.namespace("lora_sx126x")

# empty_component_ns = cg.esphome_ns.namespace('empty_component')
LoraSX126X = lora_sx126x_ns.class_('LoraSX126X', cg.Component)

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

# LoRa Radio Parameters
CONF_RF_FREQUENCY          = 915000000  # Hz
CONF_TX_OUTPUT_POWER       = 22         # dBm
CONF_LORA_BANDWIDTH        = 0	        # [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
CONF_LORA_SPREADING_FACTOR = 7          # [SF7..SF12]
CONF_LORA_CODINGRATE       = 1	        # [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
CONF_LORA_PREAMBLE_LENGTH  = 8          # Same for Tx and Rx
CONF_LORA_SYMBOL_TIMEOUT   = 0          # Symbols
CONF_LORA_FIX_LENGTH_PAYLOAD_ON = 0     # Default: False (0)
CONF_LORA_IQ_INVERSION_ON       = 0     # Default: False (0)
CONF_RX_TIMEOUT_VALUE      = 3000
CONF_TX_TIMEOUT_VALUE      = 3000

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

    cv.Optional('rf_frequency', default=CONF_RF_FREQUENCY): cv.int_,

    cv.Optional('tx_output_power',            default=CONF_TX_OUTPUT_POWER):            cv.int_,
    cv.Optional('lora_bandwidth',             default=CONF_LORA_BANDWIDTH):             cv.int_,
    cv.Optional('lora_spreading_factor',      default=CONF_LORA_SPREADING_FACTOR):      cv.int_,
    cv.Optional('lora_codingrate',            default=CONF_LORA_CODINGRATE):            cv.int_,
    cv.Optional('lora_preamble_length',       default=CONF_LORA_PREAMBLE_LENGTH):       cv.int_,
    cv.Optional('lora_symbol_timeout',        default=CONF_LORA_SYMBOL_TIMEOUT):        cv.int_,
    cv.Optional('lora_fix_length_payload_on', default=CONF_LORA_FIX_LENGTH_PAYLOAD_ON): cv.int_,
    cv.Optional('lora_iq_inversion_on',       default=CONF_LORA_IQ_INVERSION_ON):       cv.int_,
    cv.Optional('rx_timeout_value',           default=CONF_RX_TIMEOUT_VALUE):           cv.int_,
    cv.Optional('tx_timeout_value',           default=CONF_TX_TIMEOUT_VALUE):           cv.int_,

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

    cg.add(var.set_rf_frequency(config['rf_frequency']))

    cg.add(var.set_tx_output_power(config['tx_output_power']))
    cg.add(var.set_lora_bandwidth(config['lora_bandwidth']))
    cg.add(var.set_lora_spreading_factor(config['lora_spreading_factor']))
    cg.add(var.set_lora_codingrate(config['lora_codingrate']))
    cg.add(var.set_lora_preamble_length(config['lora_preamble_length']))
    cg.add(var.set_lora_symbol_timeout(config['lora_symbol_timeout']))
    cg.add(var.set_lora_fix_length_payload_on(config['lora_fix_length_payload_on']))
    cg.add(var.set_lora_iq_inversion_on(config['lora_iq_inversion_on']))
    cg.add(var.set_rx_timeout_value(config['rx_timeout_value']))
    cg.add(var.set_tx_timeout_value(config['tx_timeout_value']))
