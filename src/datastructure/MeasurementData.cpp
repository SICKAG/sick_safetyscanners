#include <sick_microscan3_ros_driver/datastructure/MeasurementData.h>

namespace sick {
namespace datastructure {


MeasurementData::MeasurementData()
{

}

UINT32 MeasurementData::getNumberOfBeams() const
{
  return m_number_of_beams;
}

void MeasurementData::setNumberOfBeams(const UINT32 &number_of_beams)
{
  m_number_of_beams = number_of_beams;
}

std::vector<ScanPoint> MeasurementData::getScanPoints() const
{
  return m_scan_points;
}

void MeasurementData::addScanPoint(ScanPoint scan_point)
{
  m_scan_points.push_back(scan_point);
}



}
}

