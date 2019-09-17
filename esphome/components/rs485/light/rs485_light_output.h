#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/rs485/rs485.h"

namespace esphome {
namespace rs485 {

class RS485LightOutput : public light::LightOutput, public RS485Listener, public RS485Device, public Component {
  public:
    void dump_config() override;
    bool parse_data(const uint8_t *data, const num_t len) override;

    void set_light(light::LightState *light) { name_ = &light->get_name(); light_ = light; }

    light::LightTraits get_traits() override {
      auto traits = light::LightTraits();
      traits.set_supports_brightness(false);
      return traits;
    }

    void write_state(light::LightState *state) override {
      bool binary;
      state->current_values_as_binary(&binary);
      if (binary)
      this->turn_on();
      else
      this->turn_off();
    }

    void turn_on();
    void turn_off();

  protected:
    const std::string *name_;
    bool state_{false};
    light::LightState *light_{nullptr};

    void publish_state(bool state);


};

}  // namespace rs485
}  // namespace esphome
