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

#include <sick_safetyscanners/datastructure/FieldData.h>

namespace sick {
namespace datastructure {

FieldData::FieldData() {}

bool FieldData::getIsValid() const
{
  return m_is_valid;
}

void FieldData::setIsValid(bool is_valid)
{
  m_is_valid = is_valid;
}

std::string FieldData::getVersionCVersion() const
{
  return m_version_c_version;
}

void FieldData::setVersionCVersion(const std::string& version_c_version)
{
  m_version_c_version = version_c_version;
}

uint8_t FieldData::getVersionMajorVersionNumber() const
{
  return m_version_major_version_number;
}

void FieldData::setVersionMajorVersionNumber(const uint8_t& version_major_version_number)
{
  m_version_major_version_number = version_major_version_number;
}

uint8_t FieldData::getVersionMinorVersionNumber() const
{
  return m_version_minor_version_number;
}

void FieldData::setVersionMinorVersionNumber(const uint8_t& version_minor_version_number)
{
  m_version_minor_version_number = version_minor_version_number;
}

uint8_t FieldData::getVersionReleaseNumber() const
{
  return m_version_release_number;
}

void FieldData::setVersionReleaseNumber(const uint8_t& version_release_number)
{
  m_version_release_number = version_release_number;
}

bool FieldData::getIsDefined() const
{
  return m_is_defined;
}

void FieldData::setIsDefined(bool is_defined)
{
  m_is_defined = is_defined;
}

uint8_t FieldData::getEvalMethod() const
{
  return m_eval_method;
}

void FieldData::setEvalMethod(const uint8_t& eval_method)
{
  m_eval_method = eval_method;
}

uint16_t FieldData::getMultiSampling() const
{
  return m_multi_sampling;
}

void FieldData::setMultiSampling(const uint16_t& multi_sampling)
{
  m_multi_sampling = multi_sampling;
}

uint16_t FieldData::getObjectResolution() const
{
  return m_object_resolution;
}

void FieldData::setObjectResolution(const uint16_t& object_resolution)
{
  m_object_resolution = object_resolution;
}

uint16_t FieldData::getFieldSetIndex() const
{
  return m_field_set_index;
}

void FieldData::setFieldSetIndex(const uint16_t& field_set_index)
{
  m_field_set_index = field_set_index;
}

uint32_t FieldData::getNameLength() const
{
  return m_name_length;
}

void FieldData::setNameLength(const uint32_t& name_length)
{
  m_name_length = name_length;
}

std::string FieldData::getFieldName() const
{
  return m_field_name;
}

void FieldData::setFieldName(const std::string& field_name)
{
  m_field_name = field_name;
}


bool FieldData::getIsWarningField() const
{
  return m_is_warning_field;
}

void FieldData::setIsWarningField(bool is_warning_field)
{
  m_is_warning_field = is_warning_field;
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

void FieldData::setBeamDistances(const std::vector<uint16_t>& beam_distances)
{
  m_beam_distances = beam_distances;
}

float FieldData::getStartAngle() const
{
  return m_start_angle;
}

void FieldData::setStartAngle(const int32_t& start_angle)
{
  m_start_angle = (float)start_angle / m_ANGLE_RESOLUTION;
}

void FieldData::setStartAngleDegrees(const float& start_angle)
{
  m_start_angle = start_angle;
}

float FieldData::getEndAngle() const
{
  return m_end_angle;
}

void FieldData::setEndAngle(const int32_t& end_angle)
{
  m_end_angle = (float)end_angle / m_ANGLE_RESOLUTION;
}

void FieldData::setEndAngleDegrees(const float& end_angle)
{
  m_end_angle = end_angle;
}

float FieldData::getAngularBeamResolution() const
{
  return m_angular_beam_resolution;
}

void FieldData::setAngularBeamResolution(const int32_t& angular_beam_resolution)
{
  m_angular_beam_resolution = (float)angular_beam_resolution / m_ANGLE_RESOLUTION;
}

void FieldData::setAngularBeamResolutionDegrees(const float& angular_beam_resolution)
{
  m_angular_beam_resolution = angular_beam_resolution;
}

} // namespace datastructure
} // namespace sick
