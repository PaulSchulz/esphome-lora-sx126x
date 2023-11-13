// -*- c++ -*-
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <atomic>
// #include "esphome/componets/text_sensor/text_sensor.h"
// #include "esphome/components/spi/spi.h"

namespace esphome {
namespace lora_sx126x {

class LoraSX126X : public sensor::Sensor, public Component {
    //    class LoraSX126X : public Component {
public:
    void setup() override;
    void loop() override;
    void dump_config() override;

    // void set_device_id(uint8_t *device_id);
    void set_frequency (uint32_t frequency)            { this->frequency_ = frequency; }
    void set_tx_output_power (uint8_t tx_output_power) { this->tx_output_power_ = tx_output_power; }
    void set_lora_bandwidth (uint8_t lora_bandwidth)   { this->lora_bandwidth_ = lora_bandwidth; }
    void set_lora_spreading_factor (uint8_t lora_spreading_factor) {
        this->lora_spreading_factor_ = lora_spreading_factor; }
    void set_lora_codingrate (int8_t lora_codingrate)   { this->lora_codingrate_ = lora_codingrate; }
    void set_lora_preamble_length (uint8_t lora_preamble_length) {
        this->lora_preamble_length_ = lora_preamble_length; }
    void set_lora_symbol_timeout (int8_t lora_symbol_timeout) {
        this->lora_symbol_timeout_ = lora_symbol_timeout; }
    void set_lora_fix_length_payload_on (int8_t lora_fix_length_payload_on) {
        this->lora_fix_length_payload_on_ = lora_fix_length_payload_on; }
    void set_lora_iq_inversion_on (int8_t lora_iq_inversion_on) {
        this->lora_iq_inversion_on_ = lora_iq_inversion_on; }
    void set_rx_timeout_value (int16_t rx_timeout_value) { this->rx_timeout_value_ = rx_timeout_value; }
    void set_tx_timeout_value (int16_t tx_timeout_value) { this->tx_timeout_value_ = tx_timeout_value; }

protected:
    uint8_t  deviceId[8];
    uint32_t frequency_;
    uint8_t  tx_output_power_;
    uint8_t  lora_bandwidth_;
    uint8_t  lora_spreading_factor_;
    uint8_t  lora_codingrate_;
    uint8_t  lora_preamble_length_;
    uint8_t  lora_symbol_timeout_;
    uint8_t  lora_fix_length_payload_on_;
    uint8_t  lora_iq_inversion_on_;
    uint16_t rx_timeout_value_;
    uint16_t tx_timeout_value_;

};



}  // namespace lora_sx126x
}  // namespace esphome
