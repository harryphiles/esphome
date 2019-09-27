#pragma once

#include "rs485.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace rs485 {

template<typename... Ts> class RS485WriteAction : public Action<Ts...>, public Parented<RS485Component> {
 public:
  void set_data_template(std::function<std::vector<uint8_t>(Ts...)> func) {
    this->data_func_ = func;
    this->static_ = false;
  }
  void set_data_static(const std::vector<uint8_t> &data) {
    this->data_static_ = {data, {}};
    this->static_ = true;
  }

  void play(Ts... x) override {
    if (this->static_) {
      this->parent_->write_with_header({nullptr, &this->data_static_});
    } else {
      std::vector<uint8_t> val = this->data_func_(x...);
      data_static_ = {val, {}};
      this->parent_->write_with_header({nullptr, &this->data_static_});
    }
  }

 protected:
  bool static_{false};
  std::function<std::vector<uint8_t>(Ts...)> data_func_{};
  cmd_hex_t data_static_{};
};

}  // namespace rs485
}  // namespace esphome
