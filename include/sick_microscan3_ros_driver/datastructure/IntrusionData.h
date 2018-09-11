#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <sick_microscan3_ros_driver/datastructure/IntrusionDatum.h>

namespace sick {
namespace datastructure {


class IntrusionData
{
public:
  IntrusionData();
  std::vector<IntrusionDatum> getIntrusionData() const;
  void setIntrusionData(const std::vector<IntrusionDatum> &intrusion_data);

private:
  std::vector<IntrusionDatum> m_intrusion_data;
};

}
}
