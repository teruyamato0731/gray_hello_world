#ifndef RCT_C610_HPP
#define RCT_C610_HPP
/// @file
/// @brief Provides the C610 class for controlling the motor driver for M3508.
/// @copyright Copyright (c) 2024 Yoshikawa Teru
/// @license This project is released under the MIT License.

#include <array>
#include "CANMessage.hpp"

/// @brief The packet structure of the C610 motor driver.
struct C610Packet {
  uint16_t angle;
  int16_t rpm;
  int16_t ampere;
};

/// @brief The C610 motor driver class for M3508.
struct C610 {
  static constexpr int max = 10000;

  void set_current(float current) {
    raw_current_ = max / 10 * current;
  }
  void set_raw_current(int16_t raw_current) {
    raw_current_ = raw_current;
  }
  uint16_t get_angle() {
    return rx_.angle;
  }
  int16_t get_rpm() {
    return rx_.rpm;
  }
  int16_t get_ampere() {
    return rx_.ampere;
  }
  int16_t get_raw_current() const {
    return raw_current_;
  }
  void parse(const uint8_t data[8]) {
    rx_.angle = uint16_t(data[0] << 8 | data[1]);
    rx_.rpm = int16_t(data[2] << 8 | data[3]);
    rx_.ampere = int16_t(data[4] << 8 | data[5]);
  }

 private:
  int16_t raw_current_ = {};
  C610Packet rx_ = {};
};

/// @brief The C610 motor driver array for M3508.
struct C610Array {
  void parse_packet(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8 && 0x200 < msg.id && msg.id <= 0x208) {
      arr_[msg.id - 0x201u].parse(msg.data);
    }
  }
  auto to_msgs() -> std::array<CANMessage, 2> const {
    uint8_t buf[16];
    for(int i = 0; i < 8; i++) {
      buf[2 * i] = arr_[i].get_raw_current() >> 8;
      buf[2 * i + 1] = arr_[i].get_raw_current() & 0xff;
    }
    return {CANMessage{0x200, buf}, CANMessage{0x1FF, buf + 8}};
  }
  auto& operator[](int index) {
    return arr_[index];
  }
  auto begin() {
    return std::begin(arr_);
  }
  auto end() {
    return std::end(arr_);
  }

 private:
  C610 arr_[8] = {};
};

#endif  /// RCT_C610_HPP
