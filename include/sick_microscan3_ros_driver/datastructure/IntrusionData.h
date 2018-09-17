#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <sick_microscan3_ros_driver/datastructure/IntrusionDatum.h>

namespace sick {
namespace datastructure {


class IntrusionData
{
public:
  IntrusionData();
  std::vector<IntrusionDatum> getIntrusionDataVector() const;
  void setIntrusionDataVector(const std::vector<IntrusionDatum> &intrusion_data_vector);

  bool isEmpty() const;
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;
  std::vector<IntrusionDatum> m_intrusion_data_vector;
};

}
}
