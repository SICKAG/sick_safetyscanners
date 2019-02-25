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
 * \file ConfigData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-02-25
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/ConfigData.h>

namespace sick {
namespace datastructure {

ConfigData::ConfigData() {}

float ConfigData::getStartAngle() const
{
  return m_start_angle;
}

void ConfigData::setStartAngle(const int32_t& start_angle)
{
  m_start_angle = (float)start_angle / ANGLE_RESOLUTION;
}

void ConfigData::setStartAngleDegrees(const float& start_angle)
{
  m_start_angle = start_angle;
}

float ConfigData::getEndAngle() const
{
  return m_end_angle;
}

void ConfigData::setEndAngle(const int32_t& end_angle)
{
  m_end_angle = (float)end_angle / ANGLE_RESOLUTION;
}

void ConfigData::setEndAngleDegrees(const float& end_angle)
{
  m_end_angle = end_angle;
}

float ConfigData::getAngularBeamResolution() const
{
  return m_angular_beam_resolution;
}

void ConfigData::setAngularBeamResolution(const int32_t& angular_beam_resolution)
{
  m_angular_beam_resolution = (float)angular_beam_resolution / ANGLE_RESOLUTION;
}

void ConfigData::setAngularBeamResolutionDegrees(const float& angular_beam_resolution)
{
  m_angular_beam_resolution = angular_beam_resolution;
}

} // namespace datastructure
} // namespace sick
