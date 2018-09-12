#include <sick_microscan3_ros_driver/data_processing/UDPPacketMerger.h>

namespace sick {
namespace data_processing {

UDPPaketMerger::UDPPaketMerger()
  : m_is_complete(false)
  , m_deployed_paket_buffer()
{

}


bool UDPPaketMerger::isComplete()
{
   return m_is_complete;
}

sick::datastructure::PacketBuffer UDPPaketMerger::getDeployedPacketBuffer()
{
  m_is_complete = false;
  return m_deployed_paket_buffer;
}

bool UDPPaketMerger::addUDPPaket(sick::datastructure::PacketBuffer buffer)
{
  if(isComplete()) {
      m_is_complete = false;
  }
  sick::datastructure::DatagramHeader datagram_header;
  sick::data_processing::ParseDatagramHeader datagram_header_parser;
  datagram_header_parser.parseUDPSequence(buffer, datagram_header);
  addToMap(buffer, datagram_header);
  deployPacketIfComplete(datagram_header);

  return isComplete();
}

bool UDPPaketMerger::addToMap(sick::datastructure::PacketBuffer buffer, sick::datastructure::DatagramHeader header)
{
  sick::datastructure::ParsedPacketBuffer parsed_packet_buffer(buffer,header);
  auto it = m_parsed_packet_buffer_map.find(header.getIdentification());
  if(it != m_parsed_packet_buffer_map.end()) {
    it->second.push_back(parsed_packet_buffer);
  }
  else
  {
    sick::datastructure::ParsedPacketBufferVector vec;
    vec.push_back(parsed_packet_buffer);
    m_parsed_packet_buffer_map[header.getIdentification()] = vec;
  }

}

bool UDPPaketMerger::deployPacketIfComplete(sick::datastructure::DatagramHeader header)
{
  auto it = m_parsed_packet_buffer_map.find(header.getIdentification());

  if (it == m_parsed_packet_buffer_map.end())
  {
    return false;
  }
  if(!checkIfComplete(header)) return false;

  sick::datastructure::ParsedPacketBufferVector vec = getSortedParsedPacketBufferForIdentification(header);
  sick::datastructure::PacketBuffer::VectorBuffer headerless_packet_buffer = removeHeaderFromParsedPacketBuffer(vec);
  m_deployed_paket_buffer.setBuffer(headerless_packet_buffer);
  return true;

}

bool UDPPaketMerger::checkIfComplete(sick::datastructure::DatagramHeader &header)
{
  UINT32 total_length = header.getTotalLength();
  sick::datastructure::ParsedPacketBufferVector vec = getSortedParsedPacketBufferForIdentification(header);
  UINT32 cur_length = calcualteCurrentLengthOfParsedPacketBuffer(vec);
  if (cur_length != total_length) {
    return false;
  }
  m_is_complete = true;
  return true;
}

UINT32 UDPPaketMerger::calcualteCurrentLengthOfParsedPacketBuffer(sick::datastructure::ParsedPacketBufferVector &vec)
{
  UINT32 cur_length = 0;

  for (auto &parsed_packet_buffer : vec)
  {
    sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.m_packet_buffer;
    cur_length += (packet_buffer.getLength()  - sick::datastructure::DatagramHeader::HEADER_SIZE);
  }
  return cur_length;
}

sick::datastructure::ParsedPacketBufferVector UDPPaketMerger::getSortedParsedPacketBufferForIdentification(sick::datastructure::DatagramHeader &header)
{
  auto it = m_parsed_packet_buffer_map.find(header.getIdentification());
  sick::datastructure::ParsedPacketBufferVector vec = it->second;
  std::sort(vec.begin(), vec.end(), sick::datastructure::sortForIncreasingOffset);
  return vec;
}

sick::datastructure::PacketBuffer::VectorBuffer UDPPaketMerger::removeHeaderFromParsedPacketBuffer(sick::datastructure::ParsedPacketBufferVector &vec)
{
  sick::datastructure::PacketBuffer::VectorBuffer headerless_packet_buffer;
  for (auto &parsed_packet_buffer : vec) {

    sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.m_packet_buffer;

    headerless_packet_buffer.insert(headerless_packet_buffer.end(),
                                    packet_buffer.getBuffer().begin() + sick::datastructure::DatagramHeader::HEADER_SIZE,
                                    packet_buffer.getBuffer().end());
  }
  return headerless_packet_buffer;
}


}
}
