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
  //TODO
  if(isComplete()) {
      m_is_complete = false;
  }

  sick::datastructure::DatagramHeader datagram_header;
  // parse Custom Header
  sick::data_processing::ParseDatagramHeader datagram_header_parser;
  datagram_header_parser.parseUDPSequence(buffer, datagram_header);
//  sick::data_processing::ParseDatagramHeader::parseUDPSequence(buffer, datagram_header);

  std::cout << "Identifikation: " << datagram_header.getIdentification() << std::endl;

  addToMap(buffer, datagram_header);
  //add to map

  //check if complete
  deployPacketIfComplete(datagram_header);

  return isComplete();
}

bool UDPPaketMerger::addToMap(sick::datastructure::PacketBuffer buffer, sick::datastructure::DatagramHeader header)
{

  //TODO
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
  //TODO
  auto it = m_parsed_packet_buffer_map.find(header.getIdentification());

  if (it != m_parsed_packet_buffer_map.end()) {
    UINT32 cur_length = 0;
    UINT32 total_length = header.getTotalLength();

    sick::datastructure::ParsedPacketBufferVector vec = it->second;

    std::sort(vec.begin(), vec.end(), sick::datastructure::sortForIncreasingOffset);

    for (auto &parsed_packet_buffer : vec)
    {
      sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.m_packet_buffer;
      sick::datastructure::DatagramHeader datagram_header = parsed_packet_buffer.m_datagram_header;

      cur_length += (packet_buffer.getLength()  - sick::datastructure::DatagramHeader::HEADER_SIZE);
      std::cout << cur_length << std::endl;
    }

    if (cur_length != total_length) {
      std::cout << cur_length << " of " << total_length << std::endl;

      return false;
    }


    sick::datastructure::PacketBuffer::VectorBuffer headerless_packet_buffer;
    for (auto &parsed_packet_buffer : vec) {

      sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.m_packet_buffer;

      headerless_packet_buffer.insert(headerless_packet_buffer.end(),
                                      packet_buffer.getBuffer().begin() + sick::datastructure::DatagramHeader::HEADER_SIZE,
                                      packet_buffer.getBuffer().end());
      std::cout << "headerless " << headerless_packet_buffer.size() << std::endl;
    }

    m_is_complete = true;
    m_deployed_paket_buffer.setBuffer(headerless_packet_buffer);





    return true;

  }

  return false;

}



}
}
