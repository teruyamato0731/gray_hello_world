#ifndef CAN_MESSAGE_HPP
#define CAN_MESSAGE_HPP
/// @file
/// @brief Provides the CANMessage struct for CAN communication.

#include <cstring>

enum CANFormat {
  CANStandard = 0,
  CANExtended = 1,
  CANAny = 2
};
enum CANType {
  CANData   = 0,
  CANRemote = 1
};

struct CANMessage {
  unsigned long id;
  byte data[8];
  byte len;
  CANType type;
  CANFormat format;

  CANMessage();
  CANMessage(unsigned int _id, const unsigned char *_data, unsigned char _len, CANType _type, CANFormat _format);
};

CANMessage::CANMessage() {
  len    = 8U;
  type   = CANData;
  format = CANStandard;
  id     = 0U;
  memset(data, 0, 8);
}

CANMessage::CANMessage(unsigned int _id, const unsigned char *_data, unsigned char _len = 8, CANType _type = CANData, CANFormat _format = CANStandard) {
  len    = (_len > 8) ? 8 : _len;
  type   = _type;
  format = _format;
  id     = _id;
  std::memcpy(data, _data, len);
}

#endif  /// CAN_MESSAGE_HPP
