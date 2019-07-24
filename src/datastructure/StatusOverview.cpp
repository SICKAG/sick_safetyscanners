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
 * \file StatusOverview.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/StatusOverview.h>

namespace sick {
namespace datastructure {

StatusOverview::StatusOverview() {}

std::string StatusOverview::getVersionCVersion() const
{
  return m_version_c_version;
}

void StatusOverview::setVersionCVersion(const std::string& version_c_version)
{
  m_version_c_version = version_c_version;
}

uint8_t StatusOverview::getVersionMajorVersionNumber() const
{
  return m_version_major_version_number;
}

void StatusOverview::setVersionMajorVersionNumber(const uint8_t& version_major_version_number)
{
  m_version_major_version_number = version_major_version_number;
}

uint8_t StatusOverview::getVersionMinorVersionNumber() const
{
  return m_version_minor_version_number;
}

void StatusOverview::setVersionMinorVersionNumber(const uint8_t& version_minor_version_number)
{
  m_version_minor_version_number = version_minor_version_number;
}

uint8_t StatusOverview::getVersionReleaseNumber() const
{
  return m_version_release_number;
}

void StatusOverview::setVersionReleaseNumber(const uint8_t& version_release_number)
{
  m_version_release_number = version_release_number;
}

uint8_t StatusOverview::getDeviceState() const
{
  return m_device_state;
}

void StatusOverview::setDeviceState(uint8_t device_state)
{
  m_device_state = device_state;
}

uint8_t StatusOverview::getConfigState() const
{
  return m_config_state;
}

void StatusOverview::setConfigState(uint8_t config_state)
{
  m_config_state = config_state;
}

uint8_t StatusOverview::getApplicationState() const
{
  return m_application_state;
}

void StatusOverview::setApplicationState(uint8_t application_state)
{
  m_application_state = application_state;
}

uint32_t StatusOverview::getCurrentTimePowerOnCount() const
{
  return m_current_time_power_on_count;
}

void StatusOverview::setCurrentTimePowerOnCount(uint32_t current_time_power_on_count)
{
  m_current_time_power_on_count = current_time_power_on_count;
}

uint32_t StatusOverview::getCurrentTimeTime() const
{
  return m_current_time_time;
}

void StatusOverview::setCurrentTimeTime(uint32_t current_time_time)
{
  m_current_time_time = current_time_time;
}

uint16_t StatusOverview::getCurrentTimeDate() const
{
  return m_current_time_date;
}

void StatusOverview::setCurrentTimeDate(uint16_t current_time_date)
{
  m_current_time_date = current_time_date;
}

uint32_t StatusOverview::getErrorInfoCode() const
{
  return m_error_info_code;
}

void StatusOverview::setErrorInfoCode(uint32_t error_info_code)
{
  m_error_info_code = error_info_code;
}

uint32_t StatusOverview::getErrorInfoTime() const
{
  return m_error_info_time;
}

void StatusOverview::setErrorInfoTime(uint32_t error_info_time)
{
  m_error_info_time = error_info_time;
}

uint16_t StatusOverview::getErrorInfoDate() const
{
  return m_error_info_date;
}

void StatusOverview::setErrorInfoDate(uint16_t error_info_date)
{
  m_error_info_date = error_info_date;
}

} // namespace datastructure
} // namespace sick
