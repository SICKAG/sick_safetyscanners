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

#ifndef DERIVEDVALUES_H
#define DERIVEDVALUES_H

#include <stdint.h>

namespace sick {
namespace datastructure {

const double ANGLE_RESOLUTION = 4194304.0;

class DerivedValues
{
public:
  DerivedValues();

  uint16_t getMultiplicationFactor() const;
  void setMultiplicationFactor(const uint16_t& multiplication_factor);

  uint16_t getNumberOfBeams() const;
  void setNumberOfBeams(const uint16_t& number_of_beams);

  uint16_t getScanTime() const;
  void setScanTime(const uint16_t& scan_time);

  float getStartAngle() const;
  void setStartAngle(const int32_t& start_angle);

  float getAngularBeamResolution() const;
  void setAngularBeamResolution(const int32_t& angular_beam_resolution);

  uint32_t getInterbeamPeriod() const;
  void setInterbeamPeriod(const uint32_t& interbeam_period);

  bool isEmpty() const;
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;

  uint16_t m_multiplication_factor;
  uint16_t m_number_of_beams;
  uint16_t m_scan_time; // ms
  uint32_t unsigned_test_start_angle;
  float m_start_angle;
  float m_angular_beam_resolution;
  uint32_t m_interbeam_period; // usecs
};

} // namespace datastructure
} // namespace sick

#endif
