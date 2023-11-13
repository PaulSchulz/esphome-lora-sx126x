#include "esphome/core/log.h"
#include "lora_sx126x.h"

#include <SX126x-Arduino.h>
#include <SPI.h>

namespace esphome {
namespace lora_sx126x {

static const char *TAG = "lora_sx126x.component";

    void LoraSX126X::setup() {
        ESP_LOGCONFIG(TAG, "LoRa SX126X Setup");

        // uint8_t deviceId[8];
        BoardGetUniqueId(deviceId);
        ESP_LOGD(TAG, "BoardId: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
                 deviceId[0],
                 deviceId[1],
                 deviceId[2],
                 deviceId[3],
                 deviceId[4],
                 deviceId[5],
                 deviceId[6],
                 deviceId[7]);

        this->publish_state(1.00);
    }

    void LoraSX126X::loop() {

    }

    void LoraSX126X::dump_config(){
        ESP_LOGCONFIG(TAG, "LoRa SX126X Config");

        ESP_LOGCONFIG(TAG, "  Frequency:          %9d Hz", frequency_);
        ESP_LOGCONFIG(TAG, "  Tx Output Power:          %3d dBm", tx_output_power_);
        ESP_LOGCONFIG(TAG, "  LoRa Bandwidth:           %3d", lora_bandwidth_);
        ESP_LOGCONFIG(TAG, "  LoRa Spreading Factor:    %3d", lora_spreading_factor_);
        ESP_LOGCONFIG(TAG, "  LoRa Codingrate:          %3d", lora_codingrate_);
        ESP_LOGCONFIG(TAG, "  LoRa Preable Length:      %3d", lora_preamble_length_);
        ESP_LOGCONFIG(TAG, "  LoRa Symbol Timeout:      %3d", lora_symbol_timeout_);
        ESP_LOGCONFIG(TAG, "  LoRa Fix Length Payload On: %d", lora_fix_length_payload_on_);
        ESP_LOGCONFIG(TAG, "  LoRa IQ Inversion On:       %d", lora_iq_inversion_on_);
        ESP_LOGCONFIG(TAG, "  Rx Timeout Value:       %5d ms", rx_timeout_value_);
        ESP_LOGCONFIG(TAG, "  Tx Timeout Value:       %5d ms", tx_timeout_value_);
    }

}  // namespace lora_sx126x
}  // namespace esphome
