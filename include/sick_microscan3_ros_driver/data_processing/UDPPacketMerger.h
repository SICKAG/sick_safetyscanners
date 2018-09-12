#pragma once

#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/data_processing/ParseDatagramHeader.h>

namespace sick {
namespace data_processing {

class UDPPaketMerger
{
public:
  UDPPaketMerger();

  bool isComplete();

  bool addUDPPaket(sick::datastructure::PacketBuffer buffer);
  sick::datastructure::PacketBuffer getDeployedPacketBuffer();
private:
  bool m_is_complete;
  sick::datastructure::PacketBuffer m_deployed_paket_buffer;

  std::map<UINT32, sick::datastructure::ParsedPacketBufferVector> m_parsed_packet_buffer_map;

  bool addToMap(sick::datastructure::PacketBuffer buffer, sick::datastructure::DatagramHeader header);
  bool deployPacketIfComplete(datastructure::DatagramHeader header);
  bool checkIfComplete(sick::datastructure::DatagramHeader &header);
  UINT32 calcualteCurrentLengthOfParsedPacketBuffer(sick::datastructure::ParsedPacketBufferVector &vec);
  sick::datastructure::ParsedPacketBufferVector getSortedParsedPacketBufferForIdentification(sick::datastructure::DatagramHeader &header);
  sick::datastructure::PacketBuffer::VectorBuffer removeHeaderFromParsedPacketBuffer(sick::datastructure::ParsedPacketBufferVector &vec);
};

}
}


