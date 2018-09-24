// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
* \file DerivedValues.h
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-09-24
*/
//----------------------------------------------------------------------

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
  UINT32 unsigned_test_start_angle;
  float m_start_angle;
  float m_angular_beam_resolution;
  UINT32 m_interbeam_period; //usecs
};

}
}
