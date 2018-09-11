#include <sick_microscan3_ros_driver/datastructure/IntrusionDatum.h>

namespace sick {
namespace datastructure {


IntrusionDatum::IntrusionDatum()
{

}

INT32 IntrusionDatum::getSize() const
{
  return m_size;

}

void IntrusionDatum::setSize(const INT32 &size)
{
  m_size = size;
}

std::vector<bool> IntrusionDatum::getFlags() const
{
  return m_flags;
}

void IntrusionDatum::setFlags(const std::vector<bool> &flags)
{
  m_flags = flags;
}

}
}

