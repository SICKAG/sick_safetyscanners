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


}
}

