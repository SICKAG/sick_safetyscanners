#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <sick_microscan3_ros_driver/datastructure/ScanPoint.h>

namespace sick {
namespace datastructure {


class MeasurementData
{
public:
  MeasurementData();

  UINT32 getNumberOfBeams() const;
  void setNumberOfBeams(const UINT32 &number_of_beams);

  std::vector<ScanPoint> getScanPoints() const;
  void addScanPoint(ScanPoint scan_point);

private:
  UINT32 m_number_of_beams;
  std::vector<ScanPoint> m_scan_points;

};

}
}
