// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2019, SICK AG, Waldkirch
*  Copyright (C) 2019, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file FieldSets.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/FieldSets.h>

namespace sick {
namespace datastructure {

FieldSets::FieldSets() {}

std::string FieldSets::getVersionCVersion() const
{
  return m_version_c_version;
}

void FieldSets::setVersionCVersion(const std::string& version_c_version)
{
  m_version_c_version = version_c_version;
}

uint8_t FieldSets::getVersionMajorVersionNumber() const
{
  return m_version_major_version_number;
}

void FieldSets::setVersionMajorVersionNumber(const uint8_t& version_major_version_number)
{
  m_version_major_version_number = version_major_version_number;
}

uint8_t FieldSets::getVersionMinorVersionNumber() const
{
  return m_version_minor_version_number;
}

void FieldSets::setVersionMinorVersionNumber(const uint8_t& version_minor_version_number)
{
  m_version_minor_version_number = version_minor_version_number;
}

uint8_t FieldSets::getVersionReleaseNumber() const
{
  return m_version_release_number;
}

void FieldSets::setVersionReleaseNumber(const uint8_t& version_release_number)
{
  m_version_release_number = version_release_number;
}

std::vector<uint32_t> FieldSets::getNameLength() const
{
  return m_name_length;
}

void FieldSets::setNameLength(const std::vector<uint32_t>& name_length)
{
  m_name_length = name_length;
}

std::vector<std::string> FieldSets::getFieldName() const
{
  return m_field_name;
}

void FieldSets::setFieldName(const std::vector<std::string>& field_name)
{
  m_field_name = field_name;
}

std::vector<bool> FieldSets::getIsDefined() const
{
  return m_is_defined;
}

void FieldSets::setIsDefined(const std::vector<bool>& is_defined)
{
  m_is_defined = is_defined;
}

} // namespace datastructure
} // namespace sick
