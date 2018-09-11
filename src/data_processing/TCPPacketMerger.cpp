#include <sick_microscan3_ros_driver/data_processing/TCPPacketMerger.h>

namespace sick {
namespace data_processing {

TCPPaketMerger::TCPPaketMerger()
  : m_is_complete(false)
  , m_deployed_paket_buffer()
{

}


bool TCPPaketMerger::isComplete()
{
  return m_is_complete;
}

bool TCPPaketMerger::isEmpty()
{
  return m_buffer_vector.empty();
}

sick::datastructure::PacketBuffer TCPPaketMerger::getDeployedPacketBuffer()
{
  m_is_complete = false;
  return m_deployed_paket_buffer;
}

bool TCPPaketMerger::addTCPPacket(sick::datastructure::PacketBuffer buffer)
{
  if(isComplete()) {
      m_is_complete = false;
  }

  addToMap(buffer);
  deployPacketIfComplete();
  return isComplete();


}

bool TCPPaketMerger::addToMap(sick::datastructure::PacketBuffer newPacket)
{
  UINT32 currentSize = getCurrentSize();
  UINT32 remainingSize = m_targetSize - currentSize;
  m_buffer_vector.push_back(newPacket);
  if (remainingSize == newPacket.getLength())
  {
    m_is_complete = true;
  }

  return isComplete();

}

bool TCPPaketMerger::deployPacketIfComplete()
{
  //TODO
  if(isComplete()) {


    sick::datastructure::PacketBuffer::VectorBuffer headerless_packet_buffer;
    for (auto &parsed_packet_buffer : m_buffer_vector) {

      sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.getBuffer();

      headerless_packet_buffer.insert(headerless_packet_buffer.end(),
                                      packet_buffer.getBuffer().begin(),
                                      packet_buffer.getBuffer().end());
      std::cout << "headerless " << headerless_packet_buffer.size() << std::endl;
    }

    //m_is_complete = true;
    m_deployed_paket_buffer.setBuffer(headerless_packet_buffer);
    m_buffer_vector.clear();

    return true;
  }
  return false;

}

UINT32 TCPPaketMerger::getTargetSize() const
{
  return m_targetSize;
}

void TCPPaketMerger::setTargetSize(const UINT32 &targetSize)
{
  m_targetSize = targetSize;
}

UINT32 TCPPaketMerger::getCurrentSize() const
{
  size_t sum = 0;
  for (auto it_packet = m_buffer_vector.begin();
     it_packet != m_buffer_vector.end(); ++it_packet)
  {
    const auto& packet = *it_packet;
    sum += packet.getLength();
  }
  return static_cast<UINT32>(sum);
}

}
}
