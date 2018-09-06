#pragma once

#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/data_processing/ParseDatagramHeader.h>

namespace sick {
namespace data_processing {

class TCPPaketMerger
{
public:
  TCPPaketMerger();

  bool isComplete();
  bool isEmpty();

  bool addTCPPacket(sick::datastructure::PacketBuffer buffer);
  sick::datastructure::PacketBuffer getDeployedPacketBuffer();
  UINT32 getTargetSize() const;
  void setTargetSize(const UINT32 &targetSize);

private:
  bool m_is_complete;
  sick::datastructure::PacketBuffer m_deployed_paket_buffer;

  std::vector<sick::datastructure::PacketBuffer> m_buffer_vector;

  bool addToMap(sick::datastructure::PacketBuffer newPacket);
  bool deployPacketIfComplete();

  UINT32 m_targetSize;
  UINT32 getCurrentSize() const;
};

}
}


