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
 * \file ScanPoint.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/ScanPoint.h>

namespace sick {
namespace datastructure {

ScanPoint::ScanPoint() {}

ScanPoint::ScanPoint(float angle,
                     int16_t& distance,
                     uint8_t& reflectivity,
                     bool& valid_bit,
                     bool& infinite_bit,
                     bool& glare_bit,
                     bool& reflector_bit,
                     bool& contamination_bit,
                     bool& contamination_warning_bit)
  : m_angle(angle)
  , m_distance(distance)
  , m_reflectivity(reflectivity)
  , m_valid_bit(valid_bit)
  , m_infinite_bit(infinite_bit)
  , m_glare_bit(glare_bit)
  , m_reflector_bit(reflector_bit)
  , m_contamination_bit(contamination_bit)
  , m_contamination_warning_bit(contamination_warning_bit)
{
}

float ScanPoint::getAngle() const
{
  return m_angle;
}

uint16_t ScanPoint::getDistance() const
{
  return m_distance;
}

uint8_t ScanPoint::getReflectivity() const
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

} // namespace datastructure
} // namespace sick
