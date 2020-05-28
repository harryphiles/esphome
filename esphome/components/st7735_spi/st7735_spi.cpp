#include "st7735_spi.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace st7735_spi {

static const char *TAG = "st7735_spi";

void SPIST7735::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SPI ST7735...");
  this->spi_setup();
  this->dc_pin_->setup();  // OUTPUT
  
  this->init_reset_();
  delay(100);  // NOLINT
  ST7735::setup();
}

void SPIST7735::dump_config() {
  LOG_DISPLAY("", "SPI ST7735", this);
  ESP_LOGCONFIG(TAG, "  Model: %s", this->model_str_());
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_UPDATE_INTERVAL(this);
}

void SPIST7735::command(uint8_t value) {
  this->dc_pin_->digital_write(false); //pull DC low to indicate Command
  this->enable();
  this->transfer_byte(value);  //write byte - SPI library
  this->disable();
}

void SPIST7735::sendCommand(uint8_t cmd, const uint8_t* dataBytes, uint8_t numDataBytes) { 
  this->command(cmd);  //write command - SPI library 
  this->sendData(dataBytes,numDataBytes);
}

void SPIST7735::sendData(const uint8_t* dataBytes, uint8_t numDataBytes) {
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->enable();
  for (uint8_t i=0; i<numDataBytes; i++) {
    this->transfer_byte(pgm_read_byte(dataBytes++));  //write byte - SPI library
  }
  this->disable();
}

void SPIST7735::data(uint8_t value) {
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->enable();
  this->transfer_byte(value);  //write byte - SPI library
  this->disable();
}

void SPIST7735::data16(uint16_t w, size_t count){
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->enable();
  for(size_t i=0; i<count; i++) {
    this->transfer_byte(w >> 8);  //write MSB - SPI library
    this->transfer_byte(w);  //write LSB - SPI library
  }
  this->disable();
}

void SPIST7735::data32(uint32_t w){
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->enable();
  this->transfer_byte(w >> 24);  //write MSB - SPI library
  this->transfer_byte(w >> 16);  //write byte - SPI library
  this->transfer_byte(w >> 8);  //write byte - SPI library
  this->transfer_byte(w);  //write LSB - SPI library
  this->disable();
}

void HOT SPIST7735::write_display_data() {
  this->dc_pin_->digital_write(true);
  this->enable();
  this->write_array(this->buffer_, this->get_buffer_length_());
  this->disable();
}


}  // namespace ssd1325_spi
}  // namespace esphome
