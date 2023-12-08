// -*- c++ -*-
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <atomic>
// #include "esphome/componets/text_sensor/text_sensor.h"
// #include "esphome/components/spi/spi.h"

namespace esphome {
    namespace lorawan_sx126x {

        class LoRaWANSX126X : public sensor::Sensor, public Component {
        public:
            void setup() override;
            void loop() override;
            void dump_config() override;

            // Radio settings
            void set_pin_lora_reset (int16_t pin_lora_reset) { this->pin_lora_reset_ = pin_lora_reset; }
            void set_pin_lora_dio_1 (int16_t pin_lora_dio_1) { this->pin_lora_dio_1_ = pin_lora_dio_1; }
            void set_pin_lora_busy (int16_t pin_lora_busy) { this->pin_lora_busy_ = pin_lora_busy; }
            void set_pin_lora_nss (int16_t pin_lora_nss) { this->pin_lora_nss_ = pin_lora_nss; }
            void set_pin_lora_sclk (int16_t pin_lora_sclk) { this->pin_lora_sclk_ = pin_lora_sclk; }
            void set_pin_lora_miso (int16_t pin_lora_miso) { this->pin_lora_miso_ = pin_lora_miso; }
            void set_pin_lora_mosi (int16_t pin_lora_mosi) { this->pin_lora_mosi_ = pin_lora_mosi; }
            void set_radio_txen (int16_t radio_txen) { this->radio_txen_ = radio_txen; }
            void set_radio_rxen (int16_t radio_rxen) { this->radio_rxen_ = radio_rxen; }

            // void set_device_id(uint8_t *device_id);
            void set_tx_output_power (uint8_t tx_output_power) { this->tx_output_power_ = tx_output_power; }
            int16_t get_rx_timeout_value (void) { return this->rx_timeout_value_; }
            int16_t get_tx_timeout_value (void) { return this->tx_timeout_value_; }

            // TODO
            void set_lorawan_region (std::string lorawan_region) {
                this->lorawan_region_ = lorawan_region; }
            void set_lorawan_authtype (std::string lorawan_authtype) {
                this->lorawan_authtype_ = lorawan_authtype; }
            void set_lorawan_device_eui (std::string lorawan_device_id)  {
                this->lorawan_device_eui_ = lorawan_device_id; }
            void set_lorawan_app_eui (std::string lorawan_app_eui) {
                this->lorawan_app_eui = lorawan_app_eui; }
            void set_lorawan_app_key (std:string lorawan_app_key) {
                this->lorawan_app_key = lorawan_app_key; }

            void     packets_rx_zero(void) { this->lora_packets_rx_ = 0; }
            void     packets_rx_incrument(void) { this->lora_packets_rx_++; }
            uint16_t packets_rx(void) { return lora_packets_rx_; }

            void     packets_tx_zero(void) { this->lora_packets_tx_ = 0; }
            void     packets_tx_incrument(void) { this->lora_packets_tx_++; }
            uint16_t packets_tx(void) { return lora_packets_tx_; }

        protected:
            // Radio Configuration
            int8_t pin_lora_reset_;
            int8_t pin_lora_dio_1_;
            int8_t pin_lora_busy_;
            int8_t pin_lora_nss_;
            int8_t pin_lora_sclk_;
            int8_t pin_lora_miso_;
            int8_t pin_lora_mosi_;
            int8_t radio_txen_;
            int8_t radio_rxen_;

            // LoRaWAN configuration
            std::string lorawan_region_;
            std::string lorawan_authtype_;
            std::string lorawan_device_eui_;
            std::string lorawan_app_eui_;
            std::string lorawan_app_key_;

            // Statistics
            uint16_t lora_packets_rx_;
            uint16_t lora_packets_tx_;

        }; // class LoraSX126X

        class LoRaWANSX126Xrssi : public sensor::Sensor, public Component {
        public:
            void setup() override;
            void publsh (float_t rssi) { this->publish_state(rssi); }
        }; // class LoraSX126Xrssi

        class LoRaWANSX126Xpkt : public text_sensor::TextSensor, public Component {
        public:
            void setup() override;
            void publish (char * val) { this->publish_state(val); }
        }; // class LoraSX126Xpkt


    }  // namespace lora_sx126x
}  // namespace esphome
