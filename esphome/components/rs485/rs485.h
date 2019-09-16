#pragma once

#include <HardwareSerial.h>
#include <set>
#include <vector>
#include "esphome/core/esphal.h"
#include "esphome/core/component.h"

#define BUFFER_SIZE 128

namespace esphome {
namespace rs485 {

typedef unsigned short num_t;

/** 상태 반영용 HEX Struct */
struct hex_t
{
    num_t offset;
    std::vector<uint8_t> data;
};


class RS485Component;

/**
 * RS485 Listener
 * @desc 각 컴포넌트에 수신 메시지 전달
 */
class RS485Listener {
    public:
        virtual bool parse_data(const uint8_t *data, const num_t len) = 0;
        void set_parent(RS485Component *parent) { parent_ = parent; }
        void set_monitor(bool monitor) { monitor_ = monitor; }
        bool is_monitor() { return monitor_; }
        void set_command_state(std::vector<uint8_t> command_state) { command_state_ = command_state; }
        bool has_command_state() { return command_state_.size() > 0; }
        std::vector<uint8_t> *get_state_command() { return &command_state_; }

    protected:
        RS485Component *parent_{nullptr};
        bool monitor_{false};
        std::vector<uint8_t> command_state_;
};

/** 
 * RS485 Core Component
 * 
 * @param baud Baud Rate
 * @param data Data bits
 * @param parity Parity(0: No parity, 2: Even, 3: Odd)
 * @param stop Stop bits
 * @param rxWait RX Receive Timeout (mSec)
 */
class RS485Component : public PollingComponent {
    public:
        RS485Component(int baud, num_t data=8, num_t parity=0, num_t stop=1, num_t rxWait=15) {
            baud_   = baud;
            data_   = data;
            parity_ = parity;
            stop_   = stop;
            rxWait_ = rxWait;
        } 
        /** 시작부(수신시 Check, 발신시 Append) */
        void set_prefix(uint8_t prefix) { prefix_ = prefix; }

        /** 종료부(수신시 Check, 발신시 Append) */
        void set_suffix(uint8_t suffix) { suffix_ = suffix; }

        /** CheckSum8 Xor(수신시 Check, 발신시 Append) */
        void set_checksum(bool checksum) { checksum_ = checksum; }

        /** 체크섬 계산 */
        uint8_t make_checksum(const uint8_t *data, const num_t len) const;

        void dump_config() override;
        void setup() override;
        void loop() override;
        void update() override;

        void write_byte(uint8_t data);
        void write_array(const uint8_t *data, const num_t len);
        void write_array(const std::vector<uint8_t> &data) { this->write_array(&data[0], data.size()); }
        void write_with_header(const std::vector<uint8_t> &data);
        void flush();

        void register_listener(RS485Listener *listener) {
            listener->set_parent(this);
            this->listeners_.push_back(listener);
        }
        /** Checksum Lambda */
        void set_checksum_lambda(std::function<uint8_t(const uint8_t prefix, const uint8_t *data, const num_t len)> &&f) { checksum_f_ = f; checksum_ = true; }

    protected:
        std::vector<RS485Listener *> listeners_;
        optional<std::function<uint8_t(const uint8_t prefix, const uint8_t *data, const num_t len)>> checksum_f_;

        int baud_;
        num_t data_;
        num_t parity_;
        num_t stop_;
        num_t rxWait_;

        uint8_t prefix_{0x00};
        uint8_t suffix_{0x00}; 
        bool checksum_{false}; 

        /** 수신데이터 검증 */
        bool validate(const uint8_t *data, const num_t len);

};

/** uint8_t[] to hex string  */
std::string hexencode(const uint8_t *raw_data, const num_t len);

/** uint8_t[] compare */
bool compare(const uint8_t *data1, const num_t len1, const uint8_t *data2, const num_t len2, const num_t offset);

/** 패킷 캡쳐용 리스너 */
class SerialMonitor : public RS485Listener {
    public:
        SerialMonitor(hex_t filter = {}) { if(!filter.data.empty()) filters_.push_back(filter); set_monitor(true); }
        void add_filter(hex_t filter) { filters_.push_back(filter); }
        bool parse_data(const uint8_t *data, const num_t len) override;

    protected:
        std::vector<hex_t> filters_;


};

}  // namespace rs485
}  // namespace esphome