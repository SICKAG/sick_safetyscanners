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
 * \file DerivedValues.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/DerivedValues.h>

namespace sick {
namespace datastructure {

DerivedValues::DerivedValues()
  : m_is_empty(false)
{
}

uint16_t DerivedValues::getMultiplicationFactor() const
{
  return m_multiplication_factor;
}

void DerivedValues::setMultiplicationFactor(const uint16_t& multiplication_factor)
{
  m_multiplication_factor = multiplication_factor;
}

uint16_t DerivedValues::getNumberOfBeams() const
{
  return m_number_of_beams;
}

void DerivedValues::setNumberOfBeams(const uint16_t& number_of_beams)
{
  m_number_of_beams = number_of_beams;
}

uint16_t DerivedValues::getScanTime() const
{
  return m_scan_time;
}

void DerivedValues::setScanTime(const uint16_t& scan_time)
{
  m_scan_time = scan_time;
}


float DerivedValues::getStartAngle() const
{
  return m_start_angle;
}

void DerivedValues::setStartAngle(const int32_t& start_angle)
{
  m_start_angle = (float)start_angle / m_ANGLE_RESOLUTION;
}

float DerivedValues::getAngularBeamResolution() const
{
  return m_angular_beam_resolution;
}

void DerivedValues::setAngularBeamResolution(const int32_t& angular_beam_resolution)
{
  m_angular_beam_resolution = (float)angular_beam_resolution / m_ANGLE_RESOLUTION;
}

uint32_t DerivedValues::getInterbeamPeriod() const
{
  return m_interbeam_period;
}

void DerivedValues::setInterbeamPeriod(const uint32_t& interbeam_period)
{
  m_interbeam_period = interbeam_period;
}

bool DerivedValues::isEmpty() const
{
  return m_is_empty;
}

void DerivedValues::setIsEmpty(bool is_empty)
{
  m_is_empty = is_empty;
}

} // namespace datastructure
} // namespace sick
