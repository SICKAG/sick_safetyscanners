#include <sick_microscan3_ros_driver/datastructure/IntrusionData.h>

namespace sick {
namespace datastructure {

IntrusionData::IntrusionData()
  : m_is_empty(false)
{

}

std::vector<IntrusionDatum> IntrusionData::getIntrusionDataVector() const
{
  return m_intrusion_data_vector;
}

void IntrusionData::setIntrusionDataVector(const std::vector<IntrusionDatum> &intrusion_data_vector)
{
  m_intrusion_data_vector = intrusion_data_vector;
}

bool IntrusionData::isEmpty() const
{
  return m_is_empty;
}

void IntrusionData::setIsEmpty(bool is_empty)
{
  m_is_empty = is_empty;
}


}
}

