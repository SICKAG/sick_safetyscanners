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
 * \file DeviceStatus.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICESTATUS_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICESTATUS_H

#include <iostream>


namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the device status of a laser scanner.
 */
class DeviceStatus
{
public:
  enum e_sopas_device_status
  {
    E_UNKNOWN,
    E_START_UP,
    E_SERVICE_MODE,
    E_NORMAL_OPERATION,
    E_SUSPENDED_OPERATION,
    E_SERVICE_RECOMMENDED,
    E_SERVICE_REQUIRED,
    E_RECOVERABLE_ERROR,
    E_FATAL_ERROR
  };
  /*!
   * \brief Constructor of the device status.
   */
  DeviceStatus();

  /*!
   * \brief Gets the device status for the scanner.
   *
   * \returns The device status for the scanner.
   */
  uint8_t getDeviceStatus() const;
  /*!
   * \brief Sets the device status for the scanner.
   *
   * \param device_status The device status for the scanner.
   */
  void setDeviceStatus(const uint8_t& device_status);

private:
  uint8_t m_device_status;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICESTATUS_H
