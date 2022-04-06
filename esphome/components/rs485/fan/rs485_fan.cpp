#include "rs485_fan.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rs485 {

static const char *TAG = "rs485.fan";

void RS485Fan::dump_config() {
  ESP_LOGCONFIG(TAG, "RS485 Fan '%s':", device_name_->c_str());
  dump_rs485_device_config(TAG);
  
  ESP_LOGCONFIG(TAG, "  Support Speed: %s", YESNO(this->speed_count_ > 0));
  
  if(state_speed_high_.data.size() > 0)
    ESP_LOGCONFIG(TAG, "  State Speed_high: %s, offset: %d", hexencode(&state_speed_high_.data[0], state_speed_high_.data.size()).c_str(), state_speed_high_.offset);
  if(state_speed_medium_.data.size() > 0)
    ESP_LOGCONFIG(TAG, "  State Speed_medium: %s, offset: %d", hexencode(&state_speed_medium_.data[0], state_speed_medium_.data.size()).c_str(), state_speed_medium_.offset);
  if(state_speed_low_.data.size() > 0)
    ESP_LOGCONFIG(TAG, "  State Speed_low: %s, offset: %d", hexencode(&state_speed_low_.data[0], state_speed_low_.data.size()).c_str(), state_speed_low_.offset);

  if(command_speed_high_.data.size() > 0)
    ESP_LOGCONFIG(TAG, "  Command Speed_high: %s", hexencode(&command_speed_high_.data[0], command_speed_high_.data.size()).c_str());
  if(command_speed_high_.ack.size() > 0)
    ESP_LOGCONFIG(TAG, "  Command Speed_high Ack: %s", hexencode(&command_speed_high_.ack[0], command_speed_high_.ack.size()).c_str());

  if(command_speed_medium_.data.size() > 0)
    ESP_LOGCONFIG(TAG, "  Command Speed_medium: %s", hexencode(&command_speed_medium_.data[0], command_speed_medium_.data.size()).c_str());
  if(command_speed_medium_.ack.size() > 0)
    ESP_LOGCONFIG(TAG, "  Command Speed_medium Ack: %s", hexencode(&command_speed_medium_.ack[0], command_speed_medium_.ack.size()).c_str());

  if(command_speed_low_.data.size() > 0)
    ESP_LOGCONFIG(TAG, "  Command Speed_low: %s", hexencode(&command_speed_low_.data[0], command_speed_low_.data.size()).c_str());
  if(command_speed_low_.ack.size() > 0)
    ESP_LOGCONFIG(TAG, "  Command Speed_low Ack: %s", hexencode(&command_speed_low_.ack[0], command_speed_low_.ack.size()).c_str());

}

fan::FanTraits RS485Fan::get_traits() {
  return fan::FanTraits(false, this->speed_count_ > 0, false, this->speed_count_ > 0 ? 3 : 0);
}

void RS485Fan::setup() {
  if(command_speed_low_.data.size() > 0 ) this->speed_count_++;
  if(command_speed_medium_.data.size() > 0) this->speed_count_++;
  if(command_speed_high_.data.size() > 0) this->speed_count_++;
}

void RS485Fan::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state = *call.get_state();
    ESP_LOGD(TAG, "'%s' Turning %s.", device_name_->c_str(), this->state ? "ON" : "OFF");
    write_with_header(this->state ? this->get_command_on() : this->get_command_off());
  }
  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed();
    switch (this->speed) {
      case 1:
        if(command_speed_low_.data.size() == 0) {
          ESP_LOGW(TAG, "'%s' Not support speed: LOW", device_name_->c_str());
          break;
        }
        write_with_header(&command_speed_low_);
        break;
      case 2:
        if(command_speed_medium_.data.size() == 0) {
          ESP_LOGW(TAG, "'%s' Not support speed: MEDIUM", device_name_->c_str());
          break;
        }
        write_with_header(&command_speed_medium_);
        break;
      case 3:
        if(command_speed_high_.data.size() == 0) {
          ESP_LOGW(TAG, "'%s' Not support speed: HIGH", device_name_->c_str());
          break;
        }
        write_with_header(&command_speed_high_);
        break;
      default:
        // protect from invalid input
        break;
    }
  }
  if (call.get_oscillating().has_value()) {
    this->oscillating = *call.get_oscillating();
    ESP_LOGW(TAG, "'%s' Not support oscillation", device_name_->c_str());
  }
  if (call.get_direction().has_value()) {
    this->direction = *call.get_direction();
    ESP_LOGW(TAG, "'%s' Not support direction", device_name_->c_str());
  }

  this->publish_state();
}

void RS485Fan::publish(const uint8_t *data, const num_t len) {
  // Speed high
  if(compare(&data[0], len, &state_speed_high_)) {
      if(this->speed != 3) {
        this->speed = 3;
        this->publish_state();
      }
      return;
  }
  // Speed medium
  else if(compare(&data[0], len, &state_speed_medium_)) {
      if(this->speed != 2) {
        this->speed = 2;
        this->publish_state();
      }
      return;
  }
  // Speed low
  else if(compare(&data[0], len, &state_speed_low_)) {
      if(this->speed != 1) {
        this->speed = 1;
        this->publish_state();
      }
      return;
  }
  ESP_LOGW(TAG, "'%s' State not found: %s", device_name_->c_str(), hexencode(&data[0], len).c_str());
}

}  // namespace rs485
}  // namespace esphome