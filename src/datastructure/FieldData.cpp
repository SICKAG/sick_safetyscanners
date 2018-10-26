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
 * \file FieldData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/datastructure/FieldData.h>

namespace sick {
namespace datastructure {

FieldData::FieldData() {}

uint16_t FieldData::getFieldSetIndex() const
{
  return m_field_set_index;
}

void FieldData::setFieldSetIndex(uint16_t& field_set_index)
{
  m_field_set_index = field_set_index;
}

bool FieldData::getIsWarningField() const
{
  return m_is_warning_field;
}

void FieldData::setIsWarningField(bool is_warning_field)
{
  m_is_warning_field = is_warning_field;
}

ScanPoint FieldData::getFieldGeometry() const
{
  return m_field_geometry;
}

void FieldData::setFieldGeometry(const ScanPoint& field_geometry)
{
  m_field_geometry = field_geometry;
}

bool FieldData::getIsProtectiveField() const
{
  return m_is_protective_field;
}

void FieldData::setIsProtectiveField(bool is_protective_field)
{
  m_is_protective_field = is_protective_field;
}

std::vector<uint16_t> FieldData::getBeamDistances() const
{
  return m_beam_distances;
}

void FieldData::setBeamDistances(const std::vector<uint16_t>& beam_distance)
{
  m_beam_distances = beam_distance;
}


} // namespace datastructure
} // namespace sick
