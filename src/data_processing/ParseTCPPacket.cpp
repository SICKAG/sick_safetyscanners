#include <sick_microscan3_ros_driver/data_processing/ParseTCPPacket.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseTCPPacket::ParseTCPPacket()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

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
  command.setDataVector(byteVector);

  return true;
}

UINT32 ParseTCPPacket::readSTx(datastructure::PacketBuffer &buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT32BigEndian(data_ptr);
}

UINT32 ParseTCPPacket::readLength(datastructure::PacketBuffer &buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 4);
  return m_reader_ptr->readUINT32BigEndian(data_ptr);
}

UINT8 ParseTCPPacket::readHubCntr(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 8);
  return m_reader_ptr->readUINT8BigEndian(data_ptr);
}
UINT8 ParseTCPPacket::readNoC(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 9);
  return m_reader_ptr->readUINT8BigEndian(data_ptr);
}
UINT32 ParseTCPPacket::readSessionID(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 10);
  return m_reader_ptr->readUINT32BigEndian(data_ptr);
}

UINT16 ParseTCPPacket::readRequestID(datastructure::PacketBuffer &buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 14);
  return m_reader_ptr->readUINT16BigEndian(data_ptr);
}

UINT8 ParseTCPPacket::readCommandType(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 16);
  return m_reader_ptr->readUINT8BigEndian(data_ptr);
}
UINT8 ParseTCPPacket::readCommandMode(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 17);
  return m_reader_ptr->readUINT8BigEndian(data_ptr);
}
UINT16 ParseTCPPacket::readErrorCode(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data() + 18);
  return m_reader_ptr->readUINT16BigEndian(data_ptr);
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

