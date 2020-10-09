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
 * \file ConfigMetadata.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/ConfigMetadata.h>

namespace sick {
namespace datastructure {

ConfigMetadata::ConfigMetadata() {}

std::string ConfigMetadata::getVersionCVersion() const
{
  return m_version_c_version;
}

void ConfigMetadata::setVersionCVersion(const std::string& version_c_version)
{
  m_version_c_version = version_c_version;
}

uint8_t ConfigMetadata::getVersionMajorVersionNumber() const
{
  return m_version_major_version_number;
}

void ConfigMetadata::setVersionMajorVersionNumber(const uint8_t& version_major_version_number)
{
  m_version_major_version_number = version_major_version_number;
}

uint8_t ConfigMetadata::getVersionMinorVersionNumber() const
{
  return m_version_minor_version_number;
}

void ConfigMetadata::setVersionMinorVersionNumber(const uint8_t& version_minor_version_number)
{
  m_version_minor_version_number = version_minor_version_number;
}

uint8_t ConfigMetadata::getVersionReleaseNumber() const
{
  return m_version_release_number;
}

void ConfigMetadata::setVersionReleaseNumber(const uint8_t& version_release_number)
{
  m_version_release_number = version_release_number;
}

uint16_t ConfigMetadata::getModificationTimeDate() const
{
  return m_modification_time_date;
}

void ConfigMetadata::setModificationTimeDate(const uint16_t& modification_time_date)
{
  m_modification_time_date = modification_time_date;
}

uint32_t ConfigMetadata::getModificationTimeTime() const
{
  return m_modification_time_time;
}

void ConfigMetadata::setModificationTimeTime(const uint32_t& modification_time_time)
{
  m_modification_time_time = modification_time_time;
}

uint16_t ConfigMetadata::getTransferTimeDate() const
{
  return m_transfer_time_date;
}

void ConfigMetadata::setTransferTimeDate(const uint16_t& transfer_time_date)
{
  m_transfer_time_date = transfer_time_date;
}

uint32_t ConfigMetadata::getTransferTimeTime() const
{
  return m_transfer_time_time;
}

void ConfigMetadata::setTransferTimeTime(const uint32_t& transfer_time_time)
{
  m_transfer_time_time = transfer_time_time;
}

uint32_t ConfigMetadata::getAppChecksum() const
{
  return m_app_checksum;
}

void ConfigMetadata::setAppChecksum(const uint32_t& app_checksum)
{
  m_app_checksum = app_checksum;
}

uint32_t ConfigMetadata::getOverallChecksum() const
{
  return m_overall_checksum;
}

void ConfigMetadata::setOverallChecksum(const uint32_t& overall_checksum)
{
  m_overall_checksum = overall_checksum;
}

std::vector<uint32_t> ConfigMetadata::getIntegrityHash() const
{
  return m_integrity_hash;
}

void ConfigMetadata::setIntegrityHash(const std::vector<uint32_t>& integrity_hash)
{
  m_integrity_hash = integrity_hash;
}


} // namespace datastructure
} // namespace sick
