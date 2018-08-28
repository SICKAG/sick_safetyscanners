#include "sick_microscan3_ros_driver/datastructure/PaketBuffer.h"

namespace sick {
namespace datastructure {


PaketBuffer::PaketBuffer(const PaketBuffer::VectorBuffer &buffer)
{
  setBuffer(buffer);
}

PaketBuffer::PaketBuffer(const PaketBuffer::ArrayBuffer &buffer, size_t length)
{
  setBuffer(buffer, length);
}

const PaketBuffer::VectorBuffer &PaketBuffer::getBuffer() const
{
  return m_buffer;
}

void PaketBuffer::setBuffer(const PaketBuffer::VectorBuffer &buffer)
{
  m_buffer.clear();
  m_buffer.insert(m_buffer.begin(), buffer.begin(), buffer.end());
}

void PaketBuffer::setBuffer(const PaketBuffer::ArrayBuffer &buffer, size_t length)
{
  m_buffer.clear();
  m_buffer.insert(m_buffer.begin(), buffer.data(), buffer.data() + length);
}

size_t PaketBuffer::getLength() const
{
  return m_buffer.size();
}

}
}

