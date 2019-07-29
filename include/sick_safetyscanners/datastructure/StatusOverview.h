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

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_STATUSOVERVIEW_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_STATUSOVERVIEW_H

#include <iostream>


namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the serial number of a laser scanner.
 */
class StatusOverview
{
public:
  enum e_device_state
  {
    E_NORMAL,
    E_ERROR,
    E_INITIALIZATION,
    E_SHUTDOWN,
    E_OPTICS_COVER_CALIBRATION
  };

  enum e_config_state
  {
    E_UNKNOWN,
    E_CONFIG_REQUIRED,
    E_CONFIG_IN_PROGRESS,
    E_NOT_VERIFIED,
    E_REJECTED,
    E_VERIFIED,
    E_INTERNAL_ERROR,
    E_VERIFICATION_IN_PROGRESS
  };

  enum e_application_state
  {
    E_STOPPED,
    E_STARTING,
    E_WAITING_FOR_PARTNERS,
    E_WAITING_FOR_INPUTS,
    E_STARTED,
    E_SLEEP_MODE
  };

  /*!
   * \brief Constructor of the serial number.
   */
  StatusOverview();

  /*!
   * \brief Gets the version indicator for the scanner.
   *
   * \returns The version indicator for the scanner.
   */
  std::string getVersionCVersion() const;
  /*!
   * \brief Sets the version indicator for the scanner.
   *
   * \param version_c_version The version indicator for the scanner.
   */
  void setVersionCVersion(const std::string& version_c_version);

  /*!
   * \brief Gets the major version number for the scanner.
   *
   * \returns The version indicator for the scanner.
   */
  uint8_t getVersionMajorVersionNumber() const;
  /*!
   * \brief Sets the major version number for the scanner.
   *
   * \param version_major_version_number The major version number for the scanner.
   */
  void setVersionMajorVersionNumber(const uint8_t& version_major_version_number);

  /*!
   * \brief Gets the minor version number for the scanner.
   *
   * \returns The minor version number for the scanner.
   */
  uint8_t getVersionMinorVersionNumber() const;
  /*!
   * \brief Sets the minor version number for the scanner.
   *
   * \param version_minor_version_number The minor version number for the scanner.
   */
  void setVersionMinorVersionNumber(const uint8_t& version_minor_version_number);

  /*!
   * \brief Gets the version release number for the scanner.
   *
   * \returns The version release number for the scanner.
   */
  uint8_t getVersionReleaseNumber() const;
  /*!
   * \brief Sets the version release number for the scanner.
   *
   * \param version_release_number The version release number for the scanner.
   */
  void setVersionReleaseNumber(const uint8_t& version_release_number);

  /*!
   * \brief Gets the device state for the scanner.
   *
   * \returns The device state for the scanner.
   */
  uint8_t getDeviceState() const;
  /*!
   * \brief Sets the device state for the scanner.
   *
   * \param device_state The device state for the scanner.
   */
  void setDeviceState(uint8_t device_state);

  /*!
   * \brief Gets the config state for the scanner.
   *
   * \returns The config state for the scanner.
   */
  uint8_t getConfigState() const;
  /*!
   * \brief Sets the config state for the scanner.
   *
   * \param config_state The config state for the scanner.
   */
  void setConfigState(uint8_t config_state);

  /*!
   * \brief Gets the application state for the scanner.
   *
   * \returns The application state for the scanner.
   */
  uint8_t getApplicationState() const;
  /*!
   * \brief Sets the application state for the scanner.
   *
   * \param application_state The application state for the scanner.
   */
  void setApplicationState(uint8_t application_state);

  /*!
   * \brief Gets the current time power on count for the scanner.
   *
   * \returns The current time power on count for the scanner.
   */
  uint32_t getCurrentTimePowerOnCount() const;
  /*!
   * \brief Sets the current time power on count for the scanner.
   *
   * \param current_time_power_on_count The current time power on count for the scanner.
   */
  void setCurrentTimePowerOnCount(uint32_t current_time_power_on_count);

  /*!
   * \brief Gets the current time time for the scanner.
   *
   * \returns The current time time for the scanner.
   */
  uint32_t getCurrentTimeTime() const;
  /*!
   * \brief Sets the current time time for the scanner.
   *
   * \param current_time_time The current time time for the scanner.
   */
  void setCurrentTimeTime(uint32_t current_time_time);

  /*!
   * \brief Gets the current time date for the scanner.
   *
   * \returns The current time date for the scanner.
   */
  uint16_t getCurrentTimeDate() const;
  /*!
   * \brief Sets the current time date for the scanner.
   *
   * \param current_time_date The current time date for the scanner.
   */
  void setCurrentTimeDate(uint16_t current_time_date);

  /*!
   * \brief Gets the error info code for the scanner.
   *
   * \returns The error info code for the scanner.
   */
  uint32_t getErrorInfoCode() const;
  /*!
   * \brief Sets the error info code for the scanner.
   *
   * \param error_info_code The error info code for the scanner.
   */
  void setErrorInfoCode(uint32_t error_info_code);


  /*!
   * \brief Gets the error info time for the scanner.
   *
   * \returns The error info time for the scanner.
   */
  uint32_t getErrorInfoTime() const;
  /*!
   * \brief Sets the error info time for the scanner.
   *
   * \param error_info_time The error info time for the scanner.
   */
  void setErrorInfoTime(uint32_t error_info_time);

  /*!
   * \brief Gets the error info date for the scanner.
   *
   * \returns The error info date for the scanner.
   */
  uint16_t getErrorInfoDate() const;
  /*!
   * \brief Sets the error info date for the scanner.
   *
   * \param error_info_date The error info date for the scanner.
   */
  void setErrorInfoDate(uint16_t error_info_date);


private:
  std::string m_version_c_version;
  uint8_t m_version_major_version_number;
  uint8_t m_version_minor_version_number;
  uint8_t m_version_release_number;
  uint8_t m_device_state;
  uint8_t m_config_state;
  uint8_t m_application_state;
  uint32_t m_current_time_power_on_count;
  uint32_t m_current_time_time;
  uint16_t m_current_time_date;
  uint32_t m_error_info_code;
  uint32_t m_error_info_time;
  uint16_t m_error_info_date;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_STATUSOVERVIEW_H
