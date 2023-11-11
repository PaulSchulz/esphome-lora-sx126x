import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

# CODEOWNERS = ["@paulschulz"]
AUTO_LOAD = [ "sensor"]
# MULTI_CONF = True

DEPENDANCIES = ["spi"]
CODEOWNERS = ["@paulschulz"]

lora_sx126x_ns = cg.esphome_ns.namespace("lora_sx126x")

# empty_component_ns = cg.esphome_ns.namespace('empty_component')
LoraSX126X = lora_sx126x_ns.class_('LoraSX126X', cg.Component)

CONF_RF_FREQUENCY          = 915000000  # Hz
CONF_TX_OUTPUT_POWER       = 22         # dBm
CONF_LORA_BANDWIDTH        = 0	        # [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
CONF_LORA_SPREADING_FACTOR = 7          # [SF7..SF12]
CONF_LORA_CODINGRATE       = 1	        # [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
CONF_LORA_PREAMBLE_LENGTH  = 8          # Same for Tx and Rx
CONF_LORA_SYMBOL_TIMEOUT   = 0          # Symbols
CONF_LORA_FIX_LENGTH_PAYLOAD_ON = False
CONF_LORA_IQ_INVERSION_ON       = False
CONF_RX_TIMEOUT_VALUE = 3000
CONF_TX_TIMEOUT_VALUE = 3000

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LoraSX126X)
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
