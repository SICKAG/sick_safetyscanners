#include <sick_microscan3_ros_driver/datastructure/MeasurementData.h>

namespace sick {
namespace datastructure {



ScanPoint::ScanPoint(float angle, INT16 &distance, UINT8 &reflectivity, bool &valid_bit, bool &infinite_bit,
                     bool &glare_bit, bool &reflector_bit, bool &contamination_bit, bool &contamination_warning_bit)
  : m_angle(angle)
  ,m_distance(distance)
  ,m_reflectivity(reflectivity)
  ,m_valid_bit(valid_bit)
  ,m_infinite_bit(infinite_bit)
  ,m_glare_bit(glare_bit)
  ,m_reflector_bit(reflector_bit)
  ,m_contamination_bit(contamination_bit)
  ,m_contamination_warning_bit(contamination_warning_bit)
{

}

float ScanPoint::getAngle() const
{
  return m_angle;
}

UINT16 ScanPoint::getDistance() const
{
  return m_distance;
}

UINT8 ScanPoint::getReflectivity() const
{
  return m_reflectivity;
}

bool ScanPoint::getValidBit() const
{
  return m_valid_bit;
}

bool ScanPoint::getInfiniteBit() const
{
  return m_infinite_bit;
}

bool ScanPoint::getGlareBit() const
{
  return m_glare_bit;
}

bool ScanPoint::getReflectorBit() const
{
  return m_reflector_bit;
}

bool ScanPoint::getContaminationBit() const
{
  return m_contamination_bit;
}

bool ScanPoint::getContaminationWarningBit() const
{
  return m_contamination_warning_bit;
}







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

