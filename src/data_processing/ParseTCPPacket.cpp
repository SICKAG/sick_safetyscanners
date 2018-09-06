#include <sick_microscan3_ros_driver/data_processing/ParseTCPPacket.h>

namespace sick {
namespace data_processing {

int ParseTCPPacket::getExpectedPacketLength(datastructure::PacketBuffer buffer)
{
  return readLength(buffer) + 8; //for STX and Length which is not included in length datafield
}

UINT16 ParseTCPPacket::getRequestID(datastructure::PacketBuffer buffer)
{
  return readRequestID(buffer);
}

bool ParseTCPPacket::parseTCPSequence(datastructure::PacketBuffer buffer, sick::cola2::Command &command)
{
  std::cout << "Beginn Parsing TCP Sequence Data" << std::endl;


  command.setSessionID(readSessionID(buffer));
  command.setRequestID(readRequestID(buffer));
  command.setCommandType(readCommandType(buffer));
  command.setCommandMode(readCommandMode(buffer));

  std::vector<BYTE> byteVector;
  readData(buffer, byteVector);
  command.setData(byteVector);

  return true;
}

UINT32 ParseTCPPacket::readSTx(datastructure::PacketBuffer &buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data());
  return sick::data_processing::ReadWriteHelper::readUINT32BE(dataPtr);
}

UINT32 ParseTCPPacket::readLength(datastructure::PacketBuffer &buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 4);
  return sick::data_processing::ReadWriteHelper::readUINT32BE(dataPtr);
}

UINT8 ParseTCPPacket::readHubCntr(datastructure::PacketBuffer& buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 8);
  return sick::data_processing::ReadWriteHelper::readUINT8BE(dataPtr);
}
UINT8 ParseTCPPacket::readNoC(datastructure::PacketBuffer& buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 9);
  return sick::data_processing::ReadWriteHelper::readUINT8BE(dataPtr);
}
UINT32 ParseTCPPacket::readSessionID(datastructure::PacketBuffer& buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 10);
  return sick::data_processing::ReadWriteHelper::readUINT32BE(dataPtr);
}

UINT16 ParseTCPPacket::readRequestID(datastructure::PacketBuffer &buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 14);
  return sick::data_processing::ReadWriteHelper::readUINT16BE(dataPtr);
}

UINT8 ParseTCPPacket::readCommandType(datastructure::PacketBuffer& buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 16);
  return sick::data_processing::ReadWriteHelper::readUINT8BE(dataPtr);
}
UINT8 ParseTCPPacket::readCommandMode(datastructure::PacketBuffer& buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 17);
  return sick::data_processing::ReadWriteHelper::readUINT8BE(dataPtr);
}
UINT16 ParseTCPPacket::readErrorCode(datastructure::PacketBuffer& buffer)
{
  const BYTE* dataPtr(buffer.getBuffer().data() + 18);
  return sick::data_processing::ReadWriteHelper::readUINT16BE(dataPtr);
}

void ParseTCPPacket::readData(datastructure::PacketBuffer& buffer, std::vector<BYTE>& byteVector)
{
  if (buffer.getLength() == 18)
  {
      // no additional data available
      std::cout << "no additional data" << std::endl;
      return;
  }
  else
  {
    byteVector.insert(byteVector.end(), buffer.getBuffer().begin() + 18, buffer.getBuffer().end());
  }
}

}
}

