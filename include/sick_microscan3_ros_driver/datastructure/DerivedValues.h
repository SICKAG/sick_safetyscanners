#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

const double ANGLE_RESOLUTION = 4194304.0;

class DerivedValues
{
public:
  DerivedValues();

  UINT16 getMultiplicationFactor() const;
  void setMultiplicationFactor(const UINT16 &multiplication_factor);

  UINT16 getNumberOfBeams() const;
  void setNumberOfBeams(const UINT16 &number_of_beams);

  UINT16 getScanTime() const;
  void setScanTime(const UINT16 &scan_time);

  float getStartAngle() const;
  void setStartAngle(const INT32 &start_angle);

  float getAngularBeamResolution() const;
  void setAngularBeamResolution(const INT32 &angular_beam_resolution);

  UINT32 getInterbeamPeriod() const;
  void setInterbeamPeriod(const UINT32 &interbeam_period);

  bool isEmpty() const;
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;

  UINT16 m_multiplication_factor;
  UINT16 m_number_of_beams;
  UINT16 m_scan_time; //ms
  //2 BYte reserved
  UINT32 unsigned_test_start_angle;
  float m_start_angle;
  float m_angular_beam_resolution;
  UINT32 m_interbeam_period; //usecs
};

}
}
