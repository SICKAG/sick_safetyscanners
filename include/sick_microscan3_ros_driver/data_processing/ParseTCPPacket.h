#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>


namespace sick {
namespace data_processing {

class ParseTCPPacket : public AbstractParseUDPSequence
{
public:
  static datastructure::IntrusionData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

  static bool parseTCPSequence(datastructure::PacketBuffer buffer, cola2::Command &command);
  static int getExpectedPacketLength(datastructure::PacketBuffer buffer);

  static UINT16 getRequestID(datastructure::PacketBuffer buffer);

private:
  ParseTCPPacket();
  static UINT32 readSTx(datastructure::PacketBuffer &buffer);
  static UINT32 readLength(datastructure::PacketBuffer &buffer);
  static UINT16 readRequestID(datastructure::PacketBuffer &buffer);
  static UINT8 readHubCntr(datastructure::PacketBuffer &buffer);
  static UINT8 readNoC(datastructure::PacketBuffer &buffer);
  static UINT32 readSessionID(datastructure::PacketBuffer &buffer);
  static UINT8 readCommandType(datastructure::PacketBuffer &buffer);
  static UINT8 readCommandMode(datastructure::PacketBuffer &buffer);
  static UINT16 readErrorCode(datastructure::PacketBuffer &buffer);
  static void readData(datastructure::PacketBuffer &buffer, std::vector<BYTE> &byteVector);
};

}
}


