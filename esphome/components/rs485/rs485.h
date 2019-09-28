#pragma once

#include <HardwareSerial.h>
#include <vector>
#include <queue>
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/esphal.h"

#define BUFFER_SIZE 128
#define MAX_TX_PANDING_TIME 3000

namespace esphome {
namespace rs485 {

typedef unsigned short num_t;
class RS485Component;
class RS485Device;
class RS485Listener;

/** State HEX Struct */
struct hex_t
{
    num_t offset;
    std::vector<uint8_t> data;
};

/** Command HEX Struct */
struct cmd_hex_t
{
    std::vector<uint8_t> data;
    std::vector<uint8_t> ack;
};

/** Send HEX Struct */
struct send_hex_t
{
    RS485Device *device;
    const cmd_hex_t *cmd;
};


/**
 * RS485 Listener
 * @desc 각 컴포넌트에 수신 메시지 전달
 */
class RS485Listener {
    public:
        virtual bool parse_data(const uint8_t *data, const num_t len) = 0;
        void set_parent(RS485Component *parent) { parent_ = parent; }

        void set_command_state(cmd_hex_t command_state) { command_state_ = command_state; }
        bool has_command_state() { return command_state_.data.size() > 0; }
        cmd_hex_t *get_command_state() { return &command_state_; }

        void set_monitor(bool monitor) { monitor_ = monitor; }
        bool is_monitor() { return monitor_; }

    protected:
        RS485Component *parent_{nullptr};
        cmd_hex_t command_state_;
        bool monitor_{false};

};


/**
 * RS485 Device
 */
class RS485Device : public RS485Listener, public Component {
    public:
        void dump_rs485_device_config(const char *TAG);

        void set_device(hex_t device) { device_ = device; }
        void set_sub_device(hex_t sub_device) { sub_device_ = sub_device; }
        void set_state_on(hex_t state_on) { state_on_ = state_on; }
        void set_state_off(hex_t state_off) { state_off_ = state_off; }
        
        void set_command_on(cmd_hex_t command_on) { command_on_ = command_on; }
        void set_command_on(std::function<cmd_hex_t()> command_on_func) { command_on_func_ = command_on_func; }
        const cmd_hex_t* get_command_on() { if(command_on_func_.has_value()) command_on_ = (*command_on_func_)(); return &command_on_; }
        
        void set_command_off(cmd_hex_t command_off) { command_off_ = command_off; }
        void set_command_off(std::function<cmd_hex_t()> command_off_func) { command_off_func_ = command_off_func; }
        const cmd_hex_t* get_command_off() { if(command_off_func_.has_value()) command_off_ = (*command_off_func_)(); return &command_off_; }

        void write_with_header(const cmd_hex_t *cmd);
        void callback() { tx_pending_ = false; tx_start_time_ = millis(); }
        
        /** RS485 raw message parse */
        bool parse_data(const uint8_t *data, const num_t len) override;

        /** Publish other message from parse_date() */
        virtual void publish(const uint8_t *data, const num_t len) = 0;

        /** Publish on/off state message from parse_date() */
        virtual bool publish(bool state) = 0;

        /** ESPHome Component loop */
        void loop() override;

        /** priority of setup(). higher -> executed earlier */
        float get_setup_priority() const override { return setup_priority::DATA; }


    protected:
        const std::string *device_name_;
        hex_t device_{};
        hex_t sub_device_{};
        hex_t state_on_{};
        hex_t state_off_{};
        cmd_hex_t command_on_{};
        optional<std::function<cmd_hex_t()>> command_on_func_{};
        cmd_hex_t command_off_{};
        optional<std::function<cmd_hex_t()>> command_off_func_{};

        unsigned long tx_start_time_{0};
        bool tx_pending_{false};
        num_t tx_retry_cnt_{0};
        const cmd_hex_t *tx_ack_waiting_{nullptr};


};


/** 
 * RS485 Core Component
 * 
 * @param baud Baud Rate
 * @param data Data bits
 * @param parity Parity(0: No parity, 2: Even, 3: Odd)
 * @param stop Stop bits
 * @param rx_wait RX Receive Timeout (mSec)
 */
class RS485Component : public PollingComponent {
    public:
        RS485Component(int baud, num_t data=8, num_t parity=0, num_t stop=1, num_t rx_wait=15) {
            baud_   = baud;
            data_   = data;
            parity_ = parity;
            stop_   = stop;
            rx_wait_ = rx_wait;
        }

        /** 시작부(수신시 Check, 발신시 Append) */
        void set_prefix(uint8_t prefix) { prefix_ = prefix; }

        /** 종료부(수신시 Check, 발신시 Append) */
        void set_suffix(uint8_t suffix) { suffix_ = suffix; }

        /** CheckSum 사용 여부 (수신시 Check, 발신시 Append) */
        void set_checksum(bool checksum) { checksum_ = checksum; }

        /** CheckSum Lambda */
        void set_checksum_lambda(std::function<uint8_t(const uint8_t prefix, const uint8_t *data, const num_t len)> &&f) { checksum_f_ = f; checksum_ = true; }

        /** CheckSum Calc */
        uint8_t make_checksum(const uint8_t *data, const num_t len) const;

        void dump_config() override;
        void setup() override;
        void loop() override;
        void update() override;
        float get_setup_priority() const override { return setup_priority::BUS; }

        void write_byte(uint8_t data);
        void write_array(const uint8_t *data, const num_t len);
        void write_array(const std::vector<uint8_t> &data) { this->write_array(&data[0], data.size()); }
        void write_with_header(const send_hex_t &send);
        void flush();

        void register_listener(RS485Listener *listener) {
            listener->set_parent(this);
            this->listeners_.push_back(listener);
        }

        /** TX Ack wait time */
        void set_tx_wait(num_t tx_wait) { tx_wait_ = tx_wait; }
        num_t get_tx_wait() { return tx_wait_; }

        /** TX Retry count */
        void set_tx_retry_cnt(num_t tx_retry_cnt) { tx_retry_cnt_ = tx_retry_cnt; }
        num_t get_tx_retry_cnt() { return tx_retry_cnt_; }

    protected:
        std::vector<RS485Listener *> listeners_{};
        optional<std::function<uint8_t(const uint8_t prefix, const uint8_t *data, const num_t len)>> checksum_f_{};

        int baud_;
        num_t data_;
        num_t parity_;
        num_t stop_;
        num_t rx_wait_;
        num_t tx_wait_{50};
        num_t tx_retry_cnt_{3};

        uint8_t prefix_{0x00};
        uint8_t suffix_{0x00};
        bool checksum_{false};

        /** 수신데이터 검증 */
        bool validate(const uint8_t *data, const num_t len);
    
    private:
        uint8_t rx_buffer_[BUFFER_SIZE]{};
        int     rx_timeOut_{rx_wait_};
        num_t   rx_bytesRead_{0};
        unsigned long rx_lastTime_{0};
        std::queue<send_hex_t> tx_queue_{};

        void rx_proc();


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