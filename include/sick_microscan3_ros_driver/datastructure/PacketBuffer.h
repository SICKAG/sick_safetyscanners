#pragma once

#include <iostream>
#include <string>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <vector>

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/datastructure/DatagramHeader.h>


namespace sick {
namespace datastructure {

const int MAXSIZE = 10000;


class PacketBuffer
{
public:

  typedef BYTE array_type;
  typedef boost::array<array_type, MAXSIZE> ArrayBuffer;
  typedef std::vector<array_type> VectorBuffer;

  PacketBuffer();
  PacketBuffer(const VectorBuffer& buffer);
  PacketBuffer(const ArrayBuffer& buffer, size_t length);

  static UINT32 getMaxSize() { return MAXSIZE;}

  const VectorBuffer& getBuffer() const;
  void setBuffer(const VectorBuffer& buffer);
  void setBuffer(const ArrayBuffer &buffer, size_t length);

  size_t getLength() const;


private:

  VectorBuffer m_buffer;
};

struct ParsedPacketBuffer
{
        ParsedPacketBuffer(const sick::datastructure::PacketBuffer& packet_buffer,
                                        sick::datastructure::DatagramHeader datagram_header)
                : m_packet_buffer(packet_buffer)
                , m_datagram_header(datagram_header)
        {
        }
        sick::datastructure::PacketBuffer m_packet_buffer;
        sick::datastructure::DatagramHeader m_datagram_header;
};
typedef std::vector<ParsedPacketBuffer> ParsedPacketBufferVector;

static bool sortForIncreasingOffset(const ParsedPacketBuffer& ppb1,
                                                         const ParsedPacketBuffer& ppb2)
{
        return ppb1.m_datagram_header.getFragmentOffset() < ppb2.m_datagram_header.getFragmentOffset();
}

}
}


