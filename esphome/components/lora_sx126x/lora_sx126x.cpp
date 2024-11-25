#include "esphome.h"

#include "lora_sx126x.h"

#include <SX126x-Arduino.h>
#include <SPI.h>

// Tag for log output.
// The following is required to use log macros outside of the 'esphome' namespace.
// See: https://github.com/esphome/issues/issues/4751
using esphome::esp_log_printf_;
static const char *TAG = "lora_sx126x";

#define BUFFER_SIZE 64 // Define the payload size here

namespace esphome {
    namespace lora_sx126x {

        hw_config hwConfig;
        static RadioEvents_t RadioEvents;
        char rxpacket[BUFFER_SIZE];

        // Object references
        LoraSX126X*     radiolib;
        LoraSX126Xrssi* radiolibrssi;
        LoraSX126Xpkt*  radiolibpkt;

        //////////////////////////////////////////////////////////////////////
        // LoRa Radio Functions
        //////////////////////////////////////////////////////////////////////`
        // OnTxDone()
        void OnTxDone(void) {
            ESP_LOGD(TAG, "OnTxDone");
            radiolib->packets_tx_incrument();
            ESP_LOGD(TAG, "Packet count: %d",radiolib->packets_tx());
            Radio.Rx(radiolib->get_rx_timeout_value());
        }

        void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
            ESP_LOGD(TAG, "OnRxDone");

            memcpy(rxpacket, payload, size);
            rxpacket[size]='\0';

            Radio.Sleep();
            ESP_LOGD(TAG, "Received packet \"%s\" with rssi:%d length:%d",rxpacket,rssi,size);

            radiolib->packets_rx_incrument();
            ESP_LOGD(TAG, "Packet count: %d",radiolib->packets_rx());

            // Publish details to Sensor API
            radiolibrssi->publish_state(1.0 * rssi);
            radiolibpkt->publish_state(rxpacket);

            // Set Radio to receive next packet
            Radio.Rx(radiolib->get_rx_timeout_value());
        }

        // OnTxTimeout()
        void OnTxTimeout(void) {
            ESP_LOGD(TAG, "OnTxTimeout");
            // Radio.Sleep();
            Radio.Rx(radiolib->get_rx_timeout_value());
        }

        // OnRxTimeout()
        void OnRxTimeout(void) {
            ESP_LOGD(TAG, "OnRxTimeout");

            // Check if our channel is available for sending
            //Radio.Standby();
            //Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
            //cadTime = millis();
            //Radio.StartCad();

            // Radio.Standby();
            Radio.Rx(radiolib->get_rx_timeout_value());
            // Radio.Rx(0);
            ESP_LOGD(TAG, "-- something has gone wrong!");
        }

        // OnRxError()
        void OnRxError(void) {
            ESP_LOGD(TAG, "OnRxError");

            // Check if our channel is available for sending
            //Radio.Standby();
            //Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
            //cadTime = millis();
            //Radio.StartCad();
            // Sending the ping will be started when the channel is free

            Radio.Rx(radiolib->get_rx_timeout_value());
        }

        // OnCadDone()
        time_t cadTime = 1000;
        void OnCadDone(bool cadResult) {
            ESP_LOGD(TAG, "OnCadDone");
            time_t duration = millis() - cadTime;
            if (cadResult) {
                ESP_LOGD(TAG, "CAD returned channel busy after %ldms\n", duration);
                Radio.Rx(radiolib->get_rx_timeout_value());
            } else {
                ESP_LOGD(TAG, "CAD returned channel free after %ldms\n", duration);
                // Radio.Send(TxdBuffer, BufferSize);
            }
        }

        //////////////////////////////////////////////////////////////////////
        // Tx() - Transmit a message

        //////////////////////////////////////////////////////////////////////
        // ESPHome Methods
        void LoraSX126X::setup() {
            int result;
            // The following is required to access object from callbacks
            radiolib = this;

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
            hwConfig.USE_DIO2_ANT_SWITCH = true;
            hwConfig.USE_DIO3_TCXO       = true;
            hwConfig.USE_DIO3_ANT_SWITCH = false;

            // Radio Statistics
            this->packets_rx_zero();
            this->packets_tx_zero();

            // Initialize the LoRa chip
            ESP_LOGD(TAG, "Calling lora_hardware_init()");
            uint32_t err_code = lora_hardware_init(hwConfig);
            if (err_code != 0) {
                ESP_LOGD(TAG, "ERROR: lora_hardware_init failed - %d\n", err_code);
            }

            // Setup LoRa Radio Mode
            ESP_LOGD(TAG, "Starting LoRa Radio Mode");
            // Initialize the Radio callbacks
            RadioEvents.TxDone    = OnTxDone;    // OnTxDone;
            RadioEvents.RxDone    = OnRxDone;    // OnRxDone;
            RadioEvents.TxTimeout = OnTxTimeout; // OnTxTimeout;
            RadioEvents.RxTimeout = OnRxTimeout; // OnRxTimeout;
            RadioEvents.RxError   = OnRxError;   // OnRxError;
            RadioEvents.CadDone   = OnCadDone;   // OnCadDone;

            // Initialize the Radio
            Radio.Init(&RadioEvents);

            // Set Radio channel
            Radio.SetChannel(rf_frequency_);

            // Set Radio RX configuration
            // See: https://github.com/beegee-tokyo/SX126x-Arduino/blob/master/src/radio/radio.h#L162
            ESP_LOGD(TAG, "Calling Radio.SetRxConfig()");
            Radio.SetRxConfig(MODEM_LORA,             // modem
                              lora_bandwidth_,
                              lora_spreading_factor_,
                              lora_codingrate_,
                              0,                      // bandwidthAfc, 0 for LoRa
                              lora_preamble_length_,
                              lora_symbol_timeout_,
                              lora_fix_length_payload_on_, // false
                              0,                      // payload length when fixed
                              true,                   // crcOn
                              0,                      // enable intra-packet frequency hopping
                              0,                      // hop period in symbols
                              lora_iq_inversion_on_,
                              true                  // receive in continious mode
                              );

            // Set Radio TX configuration
            // See: https://github.com/beegee-tokyo/SX126x-Arduino/blob/master/src/radio/radio.h#L208
            ESP_LOGD(TAG, "Calling Radio.SetTxConfig()");
            Radio.SetTxConfig(MODEM_LORA,             // modem
                              22,                     // power
                              0,                      // frequency deviation FSk only, 0 for LoRa
                              lora_bandwidth_,
                              lora_spreading_factor_, // datarate
                              lora_codingrate_,
                              lora_preamble_length_,
                              lora_fix_length_payload_on_, // false
                              true,                   // crcOn
                              0,                      // freqHopOn
                              0,                      // hopPeriod
                              lora_iq_inversion_on_,
                              1000                    // Transmission timeout
                              );


            // Start LoRa
            ESP_LOGD(TAG, "Calling Radio.Rx(%d)", rx_timeout_value_);
            Radio.Rx(rx_timeout_value_);
        }

        unsigned long previousMillis = 0;
        unsigned long interval = 10000UL;

        void LoraSX126X::loop() {
            // This will be called very often after setup time.
            // Radio.Rx(radiolib->get_rx_timeout_value());
            Radio.Rx(rx_timeout_value_);
        }

        void LoraSX126X::dump_config() {
            ESP_LOGCONFIG(TAG, "SX126X Config");
            ESP_LOGCONFIG(TAG, "  Pin LoRa Reset: %2d", pin_lora_reset_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa DIO 1: %2d", pin_lora_dio_1_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa Busy:  %2d", pin_lora_busy_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa NSS:   %2d", pin_lora_nss_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa SCLK:  %2d", pin_lora_sclk_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa MISO:  %2d", pin_lora_miso_);
            ESP_LOGCONFIG(TAG, "  Pin LoRa MOSI:  %2d", pin_lora_mosi_);
            ESP_LOGCONFIG(TAG, "  Radio TXEN:     %2d", radio_txen_);
            ESP_LOGCONFIG(TAG, "  Radio RXEN:     %2d", radio_rxen_);
            ESP_LOGCONFIG(TAG, "");

            ESP_LOGCONFIG(TAG, "LoRa Configuration");
            ESP_LOGCONFIG(TAG, "  Frequency:          %9d Hz", rf_frequency_);
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
        }

        void LoraSX126Xrssi::setup() {
            // The following is required to access object from callbacks
            radiolibrssi = this;
        }

        void LoraSX126Xpkt::setup() {
            // The following is required to access object from callbacks
            radiolibpkt = this;
        }

    }  // namespace lora_sx126x
}  // namespace esphome
