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
  static int getExpectedPacketLength(sick::datastructure::PacketBuffer buffer);
  static UINT32 readLength(datastructure::PacketBuffer buffer);
  static UINT16 readRequestID(datastructure::PacketBuffer buffer);
private:
  ParseTCPPacket();
};

}
}


