#include <sick_microscan3_ros_driver/datastructure/IntrusionData.h>

namespace sick {
namespace datastructure {

IntrusionData::IntrusionData()
{

}

std::vector<IntrusionDatum> IntrusionData::getIntrusionData() const
{
  return m_intrusion_data;
}

void IntrusionData::setIntrusionData(const std::vector<IntrusionDatum> &intrusion_data)
{
  m_intrusion_data = intrusion_data;
}

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

