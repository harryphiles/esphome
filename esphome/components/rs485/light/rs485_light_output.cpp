#include "rs485_light_output.h"
#include "esphome/core/log.h"

static const char *TAG = "rs485.light";

namespace esphome {
namespace rs485 {

  void RS485LightOutput::dump_config() {
    ESP_LOGCONFIG(TAG, "RS485 LightOutput(Binary) '%s':", name_->c_str());
    ESP_LOGCONFIG(TAG, "  Device: %s", hexencode(&device_.data[0], device_.data.size()).c_str(), device_.offset);
    ESP_LOGCONFIG(TAG, "  Sub device: %s, offset: %d", hexencode(&sub_device_.data[0], sub_device_.data.size()).c_str(), sub_device_.offset);
    ESP_LOGCONFIG(TAG, "  State ON: %s, offset: %d", hexencode(&state_on_.data[0], state_on_.data.size()).c_str(), state_on_.offset);
    ESP_LOGCONFIG(TAG, "  State OFF: %s, offset: %d", hexencode(&state_off_.data[0], state_off_.data.size()).c_str(), state_off_.offset);
    ESP_LOGCONFIG(TAG, "  Command ON: %s", hexencode(&command_on_[0], command_on_.size()).c_str());
    ESP_LOGCONFIG(TAG, "  Command OFF: %s", hexencode(&command_off_[0], command_off_.size()).c_str());
  }

  bool RS485LightOutput::parse_data(const uint8_t *data, const num_t len) {
    if(!compare(&data[0], len, &device_.data[0], device_.data.size(), device_.offset))
        return false;
    else if(sub_device_.data.size() > 0 && !compare(&data[0], len, &sub_device_.data[0], sub_device_.data.size(), sub_device_.offset))
        return false;
    
    // State OFF
    if(compare(&data[0], len, &state_off_.data[0], state_off_.data.size(), state_off_.offset)) {
        publish_state(false);
        return true;
    }
    // State ON
    else if(compare(&data[0], len, &state_on_.data[0], state_on_.data.size(), state_on_.offset)) {
        publish_state(true);
        return true;
    }

    ESP_LOGW(TAG, "'%s' State not found: %s", name_->c_str(), hexencode(&data[0], len).c_str());
    return true;
  } 

  void RS485LightOutput::turn_on() {
    if(!state_) {
        ESP_LOGD(TAG, "'%s' RS485LightOutput::turn_on", name_->c_str());
        parent_->write_with_header(command_on_);
        parent_->flush();
        state_ = true;;
    }
  }

  void RS485LightOutput::turn_off() {
    if(state_) {
        ESP_LOGD(TAG, "'%s' RS485LightOutput::turn_off", name_->c_str());
        parent_->write_with_header(command_off_);
        parent_->flush();
        state_ = false;;
    }
  }

  void RS485LightOutput::publish_state(bool state) {
    if(state == state_) return;
    else state_ = state;
    ESP_LOGD(TAG, "'%s' RS485LightOutput::publish_state(%s)", name_->c_str(), state ? "True" : "False");

    if(light_ != nullptr) light_->toggle().perform();
  }



}  // namespace rs485
}  // namespace esphome
