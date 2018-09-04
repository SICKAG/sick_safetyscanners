#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>

namespace sick {
namespace datastructure {

DerivedValues::DerivedValues()
{

}

UINT16 DerivedValues::getMultiplicationFactor() const
{
  return m_multiplication_factor;
}

void DerivedValues::setMultiplicationFactor(const UINT16 &multiplication_factor)
{
  m_multiplication_factor = multiplication_factor;
}

UINT16 DerivedValues::getNumberOfBeams() const
{
  return m_number_of_beams;
}

void DerivedValues::setNumberOfBeams(const UINT16 &number_of_beams)
{
  m_number_of_beams = number_of_beams;
}

UINT16 DerivedValues::getScanTime() const
{
  return m_scan_time;
}

void DerivedValues::setScanTime(const UINT16 &scan_time)
{
  m_scan_time = scan_time;
}

//TODO
UINT32 DerivedValues::getUnsigned_test_start_angle() const
{
  return unsigned_test_start_angle;
}

void DerivedValues::setUnsigned_test_start_angle(const UINT32 &value)
{
  unsigned_test_start_angle = value ;
}

float DerivedValues::getStartAngle() const
{
  return m_start_angle;
}

void DerivedValues::setStartAngle(const INT32 &start_angle)
{
  m_start_angle = (float) start_angle / 4194304.0;
}

float DerivedValues::getAngularBeamResolution() const
{
  return m_angular_beam_resolution;
}

void DerivedValues::setAngularBeamResolution(const INT32 &angular_beam_resolution)
{
  m_angular_beam_resolution = (float) angular_beam_resolution / 4194304.0;
}

UINT32 DerivedValues::getInterbeamPeriod() const
{
  return m_interbeam_period;
}

void DerivedValues::setInterbeamPeriod(const UINT32 &interbeam_period)
{
  m_interbeam_period = interbeam_period;
}

}
}

