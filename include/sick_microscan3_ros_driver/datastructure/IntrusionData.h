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

  std::vector<bool> getFlags() const;
  void setFlags(const std::vector<bool> &flags);

private:
  INT32 m_size;
  std::vector<bool> m_flags;
};

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
