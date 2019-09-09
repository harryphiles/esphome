#pragma once

#include "esphome/core/component.h"
#include "esphome/components/rs485/rs485.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace rs485 {

class RS485Switch : public switch_::Switch, public RS485Listener, public Component {
  public:
      void set_device(hex_t device) { device_ = device; }
      void set_sub_device(hex_t sub_device) { sub_device_ = sub_device; }
      void set_state_on(hex_t state_on) { state_on_ = state_on; }
      void set_state_off(hex_t state_off) { state_off_ = state_off; }
      void set_command_on(std::vector<uint8_t> command_on) { command_on_ = command_on; }
      void set_command_off(std::vector<uint8_t> command_off) { command_off_ = command_off; }

      void dump_config() override;
      bool parse_data(const uint8_t *data, const num_t len) override;

      void write_state(bool state) override {
          parent_->write_with_header(state ? command_on_ : command_off_);
          parent_->flush();
          publish_state(state);
      }

  protected:
      hex_t device_;
      hex_t sub_device_;
      hex_t state_on_;
      hex_t state_off_;
      std::vector<uint8_t> command_on_;
      std::vector<uint8_t> command_off_;
};

}  // namespace rs485
}  // namespace esphome