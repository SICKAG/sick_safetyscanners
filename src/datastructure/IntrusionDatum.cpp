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

std::vector<bool> IntrusionDatum::getFlagsVector() const
{
  return m_flags_vector;
}

void IntrusionDatum::setFlagsVector(const std::vector<bool> &flags_vector)
{
  m_flags_vector = flags_vector;
}

}
}

