#include <sick_microscan3_ros_driver/data_processing/ParseTCPPacket.h>

namespace sick {
namespace data_processing {

int ParseTCPPacket::getExpectedPacketLength(datastructure::PacketBuffer buffer)
{
   return 18;
}

bool ParseTCPPacket::parseTCPSequence(datastructure::PacketBuffer buffer, sick::cola2::Command &command)
{
  std::cout << "Beginn Parsing TCP Sequence Data" << std::endl;



  return true;
}

UINT32 ParseTCPPacket::readLength(datastructure::PacketBuffer buffer)
{

}

UINT16 ParseTCPPacket::readRequestID(datastructure::PacketBuffer buffer)
{

}

}
}

