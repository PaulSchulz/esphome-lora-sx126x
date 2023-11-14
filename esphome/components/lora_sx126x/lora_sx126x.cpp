#include "esphome.h"

// #include "esphome/core/log.h"
#include "lora_sx126x.h"

#include <SX126x-Arduino.h>
#include <SPI.h>

// Tag for log output.
// The following is required to use log macros outside of the 'esphome' namespace.
// See: https://github.com/esphome/issues/issues/4751
using esphome::esp_log_printf_;
static const char *TAG = "lora_sx126x.component";

#define BUFFER_SIZE 64 // Define the payload size here

hw_config hwConfig;
static RadioEvents_t RadioEvents;

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ) {
    char rxpacket[BUFFER_SIZE];
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';

    // Radio.Sleep( );
    ESP_LOGD(TAG, "Received packet \"%s\" with rssi:%d length:%d",rxpacket,rssi,size);

    // Set Radio to receive next packet
    Radio.Rx(3000);
    // Radio.Rx(rx_timeout_value_);
    // Radio.Rx(lora_sx126x::rx_timeout_value_);
}

namespace esphome {
    namespace lora_sx126x {

        void LoraSX126X::setup() {
            ESP_LOGD(TAG, "LoRa SX126X Setup (SX1262)");

            // Define the HW configuration between MCU and SX126x
            hwConfig.CHIP_TYPE      = SX1262_CHIP;	    // Example uses an eByte E22 module with an SX1262
            hwConfig.PIN_LORA_RESET = pin_lora_reset_;  // LORA RESET
            hwConfig.PIN_LORA_NSS   = pin_lora_nss_;	// LORA SPI CS
            hwConfig.PIN_LORA_SCLK  = pin_lora_sclk_;   // LORA SPI CLK
            hwConfig.PIN_LORA_MISO  = pin_lora_miso_;   // LORA SPI MISO
            hwConfig.PIN_LORA_DIO_1 = pin_lora_dio_1_;  // LORA DIO_1
            hwConfig.PIN_LORA_BUSY  = pin_lora_busy_;   // LORA SPI BUSY
            hwConfig.PIN_LORA_MOSI  = pin_lora_mosi_;   // LORA SPI MOSI
            hwConfig.RADIO_TXEN     = radio_txen_;      // LORA ANTENNA TX ENABLE
            hwConfig.RADIO_RXEN     = radio_rxen_;		// LORA ANTENNA RX ENABLE
            // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
            hwConfig.USE_DIO2_ANT_SWITCH = true;
            // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
            hwConfig.USE_DIO3_TCXO = true;
            // Only Insight ISP4520 module uses DIO3 as antenna control
            hwConfig.USE_DIO3_ANT_SWITCH = false;

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

            // Initialize the LoRa chip
            ESP_LOGD(TAG, "Calling lora_hardware_init()");
            lora_hardware_init(hwConfig);

            // Initialize the Radio callbacks
            RadioEvents.TxDone    = NULL;        // OnTxDone;
            RadioEvents.RxDone    = OnRxDone;

            RadioEvents.TxTimeout = NULL;        // OnTxTimeout;
            RadioEvents.RxTimeout = NULL;        // OnRxTimeout;
            RadioEvents.RxError   = NULL;        // OnRxError;
            RadioEvents.CadDone   = NULL;        // OnCadDone;

            // Initialize the Radio
            Radio.Init(&RadioEvents);

            // Set Radio channel
            Radio.SetChannel(frequency_);

            // Set Radio RX configuration
            ESP_LOGD(TAG, "Calling Radio.SetRxConfig()");
            Radio.SetRxConfig(MODEM_LORA, lora_bandwidth_, lora_spreading_factor_,
                              lora_codingrate_, 0, lora_preamble_length_,
                              lora_symbol_timeout_, lora_fix_length_payload_on_,
                              0, true, 0, 0, lora_iq_inversion_on_, true);

            // Start LoRa
            ESP_LOGD(TAG, "Calling Radio.Rx()");
            Radio.Rx(rx_timeout_value_);
        }

        void LoraSX126X::loop() {

        }

        void LoraSX126X::dump_config() {
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
            ESP_LOGCONFIG(TAG, "");
            ESP_LOGCONFIG(TAG, "  Pin LoRa Reset: %2d", pin_lora_reset_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa DIO 1: %2d", pin_lora_dio_1_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa Busy:  %2d", pin_lora_busy_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa NSS:   %2d", pin_lora_nss_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa SCLK:  %2d", pin_lora_sclk_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa MISO:  %2d", pin_lora_miso_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa MOSI:  %2d", pin_lora_mosi_);
            ESP_LOGCONFIG(TAG, "  Radio TXEN:     %2d", radio_txen_);
            ESP_LOGCONFIG(TAG, "  Radio RXEN:     %2d", radio_rxen_);
        }

}  // namespace lora_sx126xESP_LOGCONFIG(TAG, "  Pin LoRa Reset:  %2d", pin_lora_reset_);
}  // namespace esphome
