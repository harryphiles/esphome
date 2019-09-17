#include "rs485.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"

namespace esphome {
namespace rs485 {

static const char *TAG = "rs485";

void RS485Component::dump_config() {
    ESP_LOGCONFIG(TAG, "TTL to RS485 module");
    ESP_LOGCONFIG(TAG, "  Baud Rate: %d"          , baud_   );
    ESP_LOGCONFIG(TAG, "  Data bits: %d"          , data_   );
    ESP_LOGCONFIG(TAG, "  Parity: %d"             , parity_ );
    ESP_LOGCONFIG(TAG, "  Stop bits: %d"          , stop_   );
    ESP_LOGCONFIG(TAG, "  RX Receive Timeout: %d" , rxWait_ );
    ESP_LOGCONFIG(TAG, "  Data prefix: 0x%02X"    , prefix_ );
    ESP_LOGCONFIG(TAG, "  Data suffix: 0x%02X"    , suffix_ );
    ESP_LOGCONFIG(TAG, "  Data checksum: %s"      , YESNO(checksum_));
    ESP_LOGCONFIG(TAG, "  Listener count: %d"     , listeners_.size() );
    ESP_LOGCONFIG(TAG, "  Status request interval: %u", update_interval_ );

}

void RS485Component::setup() {
    #ifdef ARDUINO_ARCH_ESP8266
        byte serialconfig = 0x10;
    #endif
    #ifdef ARDUINO_ARCH_ESP32
        uint32_t serialconfig = 0x8000010;
    #endif

    serialconfig += parity_;
    serialconfig += (data_ - 5) << 2;
    if (stop_ == 2)
        serialconfig += 0x20;
    #ifdef ARDUINO_ARCH_ESP8266
        Serial.begin(baud_, (SerialConfig)serialconfig);
    #endif
    #ifdef ARDUINO_ARCH_ESP32
        Serial.begin(baud_, serialconfig);
    #endif

    ESP_LOGI(TAG, "HW Serial Initaialize.");
}

void RS485Component::loop() {
    uint8_t serial_buf[BUFFER_SIZE];
    int timeOut = rxWait_;
    num_t bytes_read = 0;
    while (timeOut > 0)
    {
        while (Serial.available()) {
            if (bytes_read < BUFFER_SIZE) {
                serial_buf[bytes_read] = Serial.read();
                bytes_read++;
            }
            else
                Serial.read();  // when the buffer is full, just read remaining input, but do not store...
            timeOut = rxWait_; // if serial received, reset timeout counter
        }
        delay(1);
        timeOut--;
    }
    
    if(bytes_read > 0) {
        serial_buf[bytes_read] = 0; // before logging as a char array, zero terminate the last position to be safe.

        if(!validate(&serial_buf[0], bytes_read))
            return;

        bool found = false;
        for (auto *listener : this->listeners_)
            if (listener->parse_data(&serial_buf[prefix_? 1 : 0], bytes_read-(prefix_? 1 : 0)-(suffix_? 1 : 0) )) {
                found = true;
                //if(!listener->is_monitor()) break;
            }

        #ifdef ESPHOME_LOG_HAS_VERBOSE
        if (!found) {
            ESP_LOGV(TAG, "Notfound data-> %s", hexencode(&serial_buf[0], bytes_read).c_str());
        }
        #endif

    }
}

void RS485Component::update() {
    // 현재 상태 요청
    ESP_LOGD(TAG, "RS485Component::update(): Request current state...");
}

void RS485Component::write_byte(uint8_t data) {
    Serial.write(data);
    ESP_LOGD(TAG, "Write byte-> 0x%02X", data);
}

void RS485Component::write_array(const uint8_t *data, const num_t len) {
    Serial.write(data, len);
    ESP_LOGD(TAG, "Write array-> %s", hexencode(&data[0], len).c_str());
}

void RS485Component::write_with_header(const std::vector<uint8_t> &data) {
    // Header
    if(prefix_) write_byte(prefix_);
    
    // Data part
    write_array(data);

    // XOR Checksum
    if(checksum_)
        write_byte(make_checksum(&data[0], data.size()));

    // Footer
    if(suffix_) write_byte(suffix_);
}

void RS485Component::flush() {
    ESP_LOGD(TAG, "Flushing...");
    Serial.flush();
}

bool RS485Component::validate(const uint8_t *data, const num_t len) {
    if(prefix_ && data[0] != prefix_) {
        ESP_LOGW(TAG, "[Read] Prefix not match: %s", hexencode(&data[0], len).c_str());
        return false;
    }
    if(suffix_ && data[len-1] != suffix_) {
        ESP_LOGW(TAG, "[Read] Suffix not match: %s", hexencode(&data[0], len).c_str());
        return false;
    }
    if(make_checksum(&data[0], len-(suffix_? 2 : 1)) != data[len-(suffix_? 2 : 1)]) {
        ESP_LOGW(TAG, "[Read] Checksum error: %s", hexencode(&data[0], len).c_str());
        return false;
    }
    return true;
}

uint8_t RS485Component::make_checksum(const uint8_t *data, const num_t len) const {
    if (this->checksum_f_.has_value()) {
        return (*checksum_f_)(prefix_, data[0] == prefix_ ? &data[1] : data, data[0] == prefix_ ? len-1 : len);
    }
    else {
        // CheckSum8 Xor (Default)
        uint8_t crc = data[0] == prefix_ ? 0 : prefix_;
        for(num_t i=0; i<len; i++)
            crc ^= data[i];
        return crc;
    }
}




bool SerialMonitor::parse_data(const uint8_t *data, const num_t len) {
    bool found = false;
    if(filters_.size() == 0) found = true;
    else {
        for(hex_t filter : filters_) {
            found = compare(&data[0], len, &(filter.data[0]), filter.data.size(), filter.offset);
            if(found) break;
        }
    }

    if(found)
        ESP_LOGI(TAG, "Serial Monitor: %s", hexencode(&data[0], len).c_str());
        
    return found;
}


std::string hexencode(const uint8_t *raw_data, num_t len) {
    char buf[20];
    std::string res;
    for(num_t i=0; i<len; i++) {
        sprintf(buf, "0x%02X ", raw_data[i]);
        res += buf;
    }
    sprintf(buf, "(%d byte)", len);
    res += buf;
    return res;
}

bool compare(const uint8_t *data1, const num_t len1, const uint8_t *data2, const num_t len2, const num_t offset) {
    if(len1-offset < len2)
        return false;
    //ESP_LOGD(TAG, "compare(0x%02X, 0x%02X, %d)=> %d", data1[offset], data2[0], len2, memcmp(&data1[offset], &data2[0], len2));
    return memcmp(&data1[offset], &data2[0], len2) == 0;
}


}  // namespace rs485
}  // namespace esphome