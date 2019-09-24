#pragma once

#include "esphome/core/component.h"
#include "esphome/components/rs485/rs485.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace rs485 {

class RS485Switch : public switch_::Switch, public RS485Device {
  public:
        RS485Switch() { device_name_ = &this->name_; }
        void dump_config() override;
        void publish(const uint8_t *data, const num_t len) override;
        void publish(bool state) override { publish_state(state); }

        void write_state(bool state) override {
            if(state == this->state) return;
            
            write_with_header(state ? &command_on_ : &command_off_);
            publish_state(state);
        }

  protected:

};

}  // namespace rs485
}  // namespace esphome