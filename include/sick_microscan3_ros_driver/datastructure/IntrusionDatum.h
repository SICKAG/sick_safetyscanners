#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class IntrusionDatum
{
public:
  IntrusionDatum();
  INT32 getSize() const;
  void setSize(const INT32 &size);

  std::vector<bool> getFlagsVector() const;
  void setFlagsVector(const std::vector<bool> &flags_vector);

private:
  INT32 m_size;
  std::vector<bool> m_flags_vector;
};

}
}
