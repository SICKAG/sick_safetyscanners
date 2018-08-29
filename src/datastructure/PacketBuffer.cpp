#include "sick_microscan3_ros_driver/datastructure/PacketBuffer.h"

namespace sick {
namespace datastructure {


PacketBuffer::PacketBuffer()
{
}

PacketBuffer::PacketBuffer(const PacketBuffer::VectorBuffer &buffer)
{
  setBuffer(buffer);
}

PacketBuffer::PacketBuffer(const PacketBuffer::ArrayBuffer &buffer, size_t length)
{
  setBuffer(buffer, length);
}

const PacketBuffer::VectorBuffer &PacketBuffer::getBuffer() const
{
  return m_buffer;
}

void PacketBuffer::setBuffer(const PacketBuffer::VectorBuffer &buffer)
{
  m_buffer.clear();
  m_buffer.insert(m_buffer.begin(), buffer.begin(), buffer.end());
}

void PacketBuffer::setBuffer(const PacketBuffer::ArrayBuffer &buffer, size_t length)
{
  m_buffer.clear();
  m_buffer.insert(m_buffer.begin(), buffer.data(), buffer.data() + length);
}

size_t PacketBuffer::getLength() const
{
  std::cout << "buffer: " << m_buffer.size() << std::endl;
  return m_buffer.size();
}

}
}

