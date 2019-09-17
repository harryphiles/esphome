#pragma once

#include "esphome/core/component.h"
#include "esphome/components/rs485/rs485.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace rs485 {

class RS485Switch : public switch_::Switch, public RS485Listener, public RS485Device, public Component {
  public:
        void dump_config() override;
        bool parse_data(const uint8_t *data, const num_t len) override;

        void write_state(bool state) override {
            if(state == this->state) return;
            
            parent_->write_with_header(state ? command_on_ : command_off_);
            parent_->flush();
            publish_state(state);
        }

  protected:

};

}  // namespace rs485
}  // namespace esphome