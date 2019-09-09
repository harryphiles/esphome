#include "rs485_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rs485 {

static const char *TAG = "rs485.switch";

void RS485Switch::dump_config() {
    ESP_LOGCONFIG(TAG, "RS485 Switch '%s':", this->name_.c_str());
    ESP_LOGCONFIG(TAG, "  Device: %s", hexencode(&device_.data[0], device_.data.size()).c_str(), device_.offset);
    ESP_LOGCONFIG(TAG, "  Sub device: %s, offset: %d", hexencode(&sub_device_.data[0], sub_device_.data.size()).c_str(), sub_device_.offset);
    ESP_LOGCONFIG(TAG, "  State ON: %s, offset: %d", hexencode(&state_on_.data[0], state_on_.data.size()).c_str(), state_on_.offset);
    ESP_LOGCONFIG(TAG, "  State OFF: %s, offset: %d", hexencode(&state_off_.data[0], state_off_.data.size()).c_str(), state_off_.offset);
    ESP_LOGCONFIG(TAG, "  Command ON: %s", hexencode(&command_on_[0], command_on_.size()).c_str());
    ESP_LOGCONFIG(TAG, "  Command OFF: %s", hexencode(&command_off_[0], command_off_.size()).c_str());
}

bool RS485Switch::parse_data(const uint8_t *data, const num_t len) {
    if(!compare(&data[0], len, &device_.data[0], device_.data.size(), device_.offset))
        return false;
    else if(sub_device_.data.size() > 0 && !compare(&data[0], len, &sub_device_.data[0], sub_device_.data.size(), sub_device_.offset))
        return false;
    
    if(compare(&data[0], len, &state_off_.data[0], state_off_.data.size(), state_off_.offset)) {
        publish_state(false);
        return true;
    }
    else if(compare(&data[0], len, &state_on_.data[0], state_on_.data.size(), state_on_.offset)) {
        publish_state(true);
        return true;
    }
    ESP_LOGW(TAG, "'%s' State not found: %s", this->name_.c_str(), hexencode(&data[0], len).c_str());
    return true;
}

}  // namespace uart
}  // namespace esphome
