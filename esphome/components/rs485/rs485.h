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

    protected:
        RS485Component *parent_{nullptr};
        bool monitor_{false};
};

/** 
 * RS485 Core Component
 */
class RS485Component : public Component {
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

        void write_byte(uint8_t data);
        void write_array(const uint8_t *data, const num_t len);
        void write_array(const std::vector<uint8_t> &data) { this->write_array(&data[0], data.size()); }
        void write_with_header(const std::vector<uint8_t> &data);
        void flush();

        void register_listener(RS485Listener *listener) {
            listener->set_parent(this);
            this->listeners_.push_back(listener);
        }

    protected:
        std::vector<RS485Listener *> listeners_;

        int baud_;      // Baud Rate
        num_t data_;    // Data bits
        num_t parity_;  // Parity(0: No parity, 2: Even, 3: Odd)
        num_t stop_;    // Stop bits
        num_t rxWait_;  // RX Receive Timeout (mSec)

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