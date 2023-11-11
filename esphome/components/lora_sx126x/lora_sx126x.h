// -*- c++ -*-
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
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

    protected:
        uint8_t deviceId[8];
    };



}  // namespace lora_sx126x
}  // namespace esphome
