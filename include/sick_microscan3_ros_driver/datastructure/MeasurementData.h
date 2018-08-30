#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class ScanPoint
{
public:
  ScanPoint();
  ScanPoint(float angle, INT16 &distance, UINT8 &reflectivity, bool &valid_bit, bool &infinite_bit, bool &glare_bit, bool &reflector_bit, bool &contamination_bit, bool &contamination_warning_bit);

  float getAngle() const;

  UINT16 getDistance() const;

  UINT8 getReflectivity() const;

  bool getValidBit() const;

  bool getInfiniteBit() const;

  bool getGlareBit() const;

  bool getReflectorBit() const;

  bool getContaminationBit() const;

  bool getContaminationWarningBit() const;



private:
  float m_angle;
  INT16 m_distance;
  UINT8 m_reflectivity;
  bool m_valid_bit;
  bool m_infinite_bit;
  bool m_glare_bit;
  bool m_reflector_bit;
  bool m_contamination_bit;
  bool m_contamination_warning_bit;
};


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
