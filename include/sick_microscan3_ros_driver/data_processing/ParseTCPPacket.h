#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>



namespace sick {

namespace cola2 {
class Command;
}

namespace data_processing {



class ParseTCPPacket : public AbstractParseUDPSequence
{
public:
  ParseTCPPacket();

  datastructure::IntrusionData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

  bool parseTCPSequence(datastructure::PacketBuffer buffer, sick::cola2::Command &command);
  int getExpectedPacketLength(datastructure::PacketBuffer buffer);

  UINT16 getRequestID(datastructure::PacketBuffer buffer);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_readerPtr;
  UINT32 readSTx(datastructure::PacketBuffer &buffer);
  UINT32 readLength(datastructure::PacketBuffer &buffer);
  UINT16 readRequestID(datastructure::PacketBuffer &buffer);
  UINT8 readHubCntr(datastructure::PacketBuffer &buffer);
  UINT8 readNoC(datastructure::PacketBuffer &buffer);
  UINT32 readSessionID(datastructure::PacketBuffer &buffer);
  UINT8 readCommandType(datastructure::PacketBuffer &buffer);
  UINT8 readCommandMode(datastructure::PacketBuffer &buffer);
  UINT16 readErrorCode(datastructure::PacketBuffer &buffer);
  void readData(datastructure::PacketBuffer &buffer, std::vector<BYTE> &byteVector);
};

}
}


