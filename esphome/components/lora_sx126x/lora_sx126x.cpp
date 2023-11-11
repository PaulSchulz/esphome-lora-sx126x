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
    }

    void LoraSX126X::loop() {

    }

    void LoraSX126X::dump_config(){
        ESP_LOGCONFIG(TAG, "LoRa SX126X Config");
}


}  // namespace lora_sx126x
}  // namespace esphome
