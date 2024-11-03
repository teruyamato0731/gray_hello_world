#ifndef PACKET_HPP
#define PACKET_HPP

#include <cstdint>
#include <cobs.h>

constexpr size_t SIZE_OF_CONTROL = 2;
constexpr size_t SIZE_OF_SENSOR = 16;

struct Control {
  int16_t current;

  void decode(uint8_t* data) {
    cobs::decode(data, SIZE_OF_CONTROL + 1);
    *this = *reinterpret_cast<const Control*>(data + 1);
  }
};

struct Sensor {
  int16_t encoder[2];
  float gyro;
  float acc[2];

  void encode(uint8_t* data) const {
    memcpy(data + 1, this, SIZE_OF_SENSOR);
    cobs::encode(data, SIZE_OF_SENSOR + 1);
    data[SIZE_OF_SENSOR + 1] = 0x00;
  }
};

static_assert(sizeof(Control) == SIZE_OF_CONTROL);
static_assert(sizeof(Sensor) == SIZE_OF_SENSOR);

#endif // PACKET_HPP
