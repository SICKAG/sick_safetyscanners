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
 * \file DeviceName.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICENAME_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICENAME_H

#include <iostream>


namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the device name of a laser scanner.
 */
class DeviceName
{
public:
  /*!
   * \brief Constructor of the device name.
   */
  DeviceName();

  /*!
   * \brief Gets the device name for the scanner.
   *
   * \returns The device name for the scanner.
   */
  std::string getDeviceName() const;
  /*!
   * \brief Sets the device name for the scanner.
   *
   * \param device_name The device name for the scanner.
   */
  void setDeviceName(const std::string& device_name);

private:
  std::string m_device_name;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_DEVICENAME_H
