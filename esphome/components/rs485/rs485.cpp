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
    ESP_LOGCONFIG(TAG, "  RX Receive Timeout: %d" , rx_wait_ );
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

    rx_proc();
    
    if(rx_bytesRead_ > 0) {
        rx_buffer_[rx_bytesRead_] = 0; // before logging as a char array, zero terminate the last position to be safe.

        if(!validate(&rx_buffer_[0], rx_bytesRead_))
            return;

        bool found = false;
        for (auto *listener : this->listeners_)
            if (listener->parse_data(&rx_buffer_[prefix_? 1 : 0], rx_bytesRead_-(prefix_? 1 : 0)-(suffix_? 1 : 0) )) {
                found = true;
                //if(!listener->is_monitor()) break;
            }

        #ifdef ESPHOME_LOG_HAS_VERY_VERBOSE
            ESP_LOGVV(TAG, "Recieve data-> %s, term time: %dms", hexencode(&rx_buffer_[0], rx_bytesRead_).c_str(), millis() - rx_lastTime_);
        #else
            #ifdef ESPHOME_LOG_HAS_VERBOSE
            if (!found) {
                ESP_LOGV(TAG, "Notfound data-> %s", hexencode(&rx_buffer_[0], rx_bytesRead_).c_str());
            }
            #endif
        #endif
    }
    rx_lastTime_ = millis();

    // 송신 큐
    if(!tx_queue_.empty() && !rx_bytesRead_) {
        const cmd_hex_t *cmd = tx_queue_.front().cmd;

        // Header
        if(prefix_) write_byte(prefix_);
        
        // Data part
        write_array(cmd->data);

        // XOR Checksum
        if(checksum_)
            write_byte(make_checksum(&(cmd->data)[0], (cmd->data).size()));

        // Footer
        if(suffix_) write_byte(suffix_);

        // wait for send
        flush();

        // Callback
        if(tx_queue_.front().device)
            (*tx_queue_.front().device).callback();
        tx_queue_.pop();

        rx_lastTime_ = millis();
    }
}

void RS485Component::rx_proc() {
    memset(&rx_buffer_, 0, BUFFER_SIZE) ;
    rx_timeOut_ = rx_wait_;
    rx_bytesRead_ = 0;
    while (rx_timeOut_ > 0)
    {
        while (Serial.available()) {
            if (rx_bytesRead_ < BUFFER_SIZE) {
                rx_buffer_[rx_bytesRead_] = Serial.read();
                rx_bytesRead_++;
                if(suffix_ && rx_buffer_[rx_bytesRead_-1] == suffix_) return;
            }
            else
                Serial.read();  // when the buffer is full, just read remaining input, but do not store...
            rx_timeOut_ = rx_wait_; // if serial received, reset timeout counter
        }
        delay(1);
        rx_timeOut_--;
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

void RS485Component::write_with_header(const send_hex_t &send) {
    tx_queue_.push(send);
}

void RS485Component::flush() {
    Serial.flush();
    ESP_LOGD(TAG, "Flushing... (%dms)", millis() - rx_lastTime_);
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


void RS485Device::dump_rs485_device_config(const char *TAG) {
    ESP_LOGCONFIG(TAG, "  Device: %s", hexencode(&device_.data[0], device_.data.size()).c_str(), device_.offset);
    ESP_LOGCONFIG(TAG, "  Sub device: %s, offset: %d", hexencode(&sub_device_.data[0], sub_device_.data.size()).c_str(), sub_device_.offset);
    ESP_LOGCONFIG(TAG, "  State ON: %s, offset: %d", hexencode(&state_on_.data[0], state_on_.data.size()).c_str(), state_on_.offset);
    ESP_LOGCONFIG(TAG, "  State OFF: %s, offset: %d", hexencode(&state_off_.data[0], state_off_.data.size()).c_str(), state_off_.offset);
    
    ESP_LOGCONFIG(TAG, "  Command ON: %s", hexencode(&command_on_.data[0], command_on_.data.size()).c_str());
    if(command_on_.ack.size() > 0)
        ESP_LOGCONFIG(TAG, "  Command ON Ack: %s", hexencode(&command_on_.ack[0], command_on_.ack.size()).c_str());

    ESP_LOGCONFIG(TAG, "  Command OFF: %s", hexencode(&command_off_.data[0], command_off_.data.size()).c_str());
    if(command_off_.ack.size() > 0)
        ESP_LOGCONFIG(TAG, "  Command OFF Ack: %s", hexencode(&command_off_.ack[0], command_off_.ack.size()).c_str());

    ESP_LOGCONFIG(TAG, "  Command State: %s", hexencode(&command_state_.data[0], command_state_.data.size()).c_str());
    if(command_state_.ack.size() > 0)
        ESP_LOGCONFIG(TAG, "  Command State Ack: %s", hexencode(&command_state_.ack[0], command_state_.ack.size()).c_str());
}

bool RS485Device::parse_data(const uint8_t *data, const num_t len) {
    if(tx_pending_) return false;
    if(tx_ack_waiting_) {
        if(compare(&data[0], len, &tx_ack_waiting_->ack[0], tx_ack_waiting_->ack.size(), 0)) {
            tx_ack_waiting_ = nullptr;
            tx_retry_cnt_ = 0;
            ESP_LOGD(TAG, "'%s' Ack: %s", device_name_->c_str(), hexencode(data, len).c_str());
        }
        return true;
    }

    if(!compare(&data[0], len, &device_.data[0], device_.data.size(), device_.offset))
        return false;
    else if(sub_device_.data.size() > 0 && !compare(&data[0], len, &sub_device_.data[0], sub_device_.data.size(), sub_device_.offset))
        return false;
    
    // Turn OFF Message
    if(compare(&data[0], len, &state_off_.data[0], state_off_.data.size(), state_off_.offset)) {
        publish(false);
        return true;
    }
    // Turn ON Message
    else if(compare(&data[0], len, &state_on_.data[0], state_on_.data.size(), state_on_.offset)) {
        publish(true);
        return true;
    }

    // Other Message
    publish(data, len);
    return true;
}

void RS485Device::write_with_header(const cmd_hex_t *cmd) {
    tx_pending_ = true;
    parent_->write_with_header({this, cmd});

    tx_start_time_ = millis();
    tx_ack_waiting_ = cmd->ack.size() > 0 ? cmd : nullptr;
}

void RS485Device::loop() {
    if(!tx_pending_ && tx_ack_waiting_ == nullptr) {
        return;
    }

    // 전송큐 실행 될 때까지 대기
    if(tx_pending_ && millis()-tx_start_time_ < MAX_TX_PANDING_TIME) {
        delay(5);
        return;
    }

    // 실제 전송 후 ACK 대기
    if(tx_ack_waiting_) {
        if(millis() - tx_start_time_ < parent_->get_tx_wait()) {
            delay(5);
            return;
        }

        if(tx_retry_cnt_ < parent_->get_tx_retry_cnt()) {
            tx_retry_cnt_++;
            ESP_LOGD(TAG, "'%s' Retry count: %d", device_name_->c_str(), tx_retry_cnt_);
            write_with_header(tx_ack_waiting_);
        }
        else {
            tx_ack_waiting_ = nullptr;
            tx_retry_cnt_ = 0;
        }
        
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