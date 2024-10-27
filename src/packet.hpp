#ifndef PACKET_HPP
#define PACKET_HPP

#include <cstdint>
#include <cobs.h>

constexpr size_t SIZE_OF_SENSOR = 8;

struct Sensor {
  float encoder;
  float gyro;

  void encode(uint8_t* data) const {
    memcpy(data, this, SIZE_OF_SENSOR);
    cobs::encode(data, SIZE_OF_SENSOR);
  }
};

#endif // PACKET_HPP
