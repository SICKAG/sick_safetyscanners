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
 * \file DeviceStatusVariableCommand.h
 *
 * \author  Jonathan Meyer <jmeyer1292@gmail.com>
 * \date    2019-03-11
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICESTATUS_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICESTATUS_H

#include <cstdint>

namespace sick {
namespace datastructure {

/*!
 * \brief Status information for the given sensor such as time of day and version information.
 */
class DeviceStatus
{
public:
  /*!
   * \brief The constructor of the field data.
   */
  DeviceStatus();

  uint8_t getVersionIndicator() const;
  void setVersionIndicator(uint8_t version_indicator);

  uint8_t getVersionMajorVersion() const;
  void setVersionMajorVersion(uint8_t major_version);

  uint8_t getVersionMinorVersion() const;
  void setVersionMinorVersion(uint8_t minor_version);

  uint8_t getVersionRelease() const;
  void setVersionRelease(uint8_t version_release);

  uint8_t getDeviceState() const;
  void setDeviceState(uint8_t device_state);

  uint8_t getConfigState() const;
  void setConfigState(uint8_t config_state);

  uint8_t getApplicationState() const;
  void setApplicationState(uint8_t application_state);

  uint32_t getPowerOnCount() const;
  void setPowerOnCount(uint32_t power_on_count);

  uint32_t getCurrentTimeTime() const;
  void setCurrentTimeTime(uint32_t time_time);

  uint16_t getCurrentTimeDate() const;
  void setCurrentTimeDate(uint16_t time_date);

  uint32_t getErrorInfoCode() const;
  void setErrorInfoCode(uint32_t error_info_code);

  uint32_t getErrorInfoTime() const;
  void setErrorInfoTime(uint32_t error_info_time);

  uint16_t getErrorInfoDate() const;
  void setErrorInfoDate(uint16_t error_info_date);

private:
  uint8_t m_version_indicator;
  uint8_t m_version_major_version;
  uint8_t m_version_minor_version;
  uint8_t m_version_release;
  uint8_t m_device_state;
  uint8_t m_config_state;
  uint8_t m_application_state;
  uint32_t m_power_on_count;
  uint32_t m_time_time;
  uint16_t m_time_date;
  uint32_t m_error_info_code;
  uint32_t m_error_info_time;
  uint16_t m_error_info_date;
};


} // namespace datastructure
} // namespace sick


#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICESTATUS_H
