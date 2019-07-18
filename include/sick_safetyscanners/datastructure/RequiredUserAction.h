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
 * \file RequiredUserAction.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_REQUIREDUSERACTION_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_REQUIREDUSERACTION_H

#include <iostream>


namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the additional information about the sopas state.
 */
class RequiredUserAction
{
public:
  /*!
   * \brief Constructor of the RequiredUserAction.
   */
  RequiredUserAction();

  /*!
   * \brief Gets if the configuration has to be confirmed.
   *
   * \returns If the configuration has to be confirmed.
   */
  bool getConfirmConfiguration() const;
  /*!
   * \brief Sets whether the configuration has to be confirmed.
   *
   * \param confirm_configuration Whether the configuration has to be confirmed.
   */
  void setConfirmConfiguration(bool confirm_configuration);

  /*!
   * \brief Gets if the configuration has to be checked.
   *
   * \returns If the configuration has to be checked.
   */
  bool getCheckConfiguration() const;
  /*!
   * \brief Sets whether the configuration has to be checked.
   *
   * \param check_configuration Whether the configuration has to be checked.
   */
  void setCheckConfiguration(bool check_configuration);

  /*!
   * \brief Gets if the environment has to be checked.
   *
   * \returns If the environment has to be checked.
   */
  bool getCheckEnvironment() const;
  /*!
   * \brief Sets whether the environment has to be checked.
   *
   * \param check_environment Whether the environment has to be checked.
   */
  void setCheckEnvironment(bool check_environment);

  /*!
   * \brief Gets if the application interfaces have to be checked.
   *
   * \returns If the application interfaces have to be checked.
   */
  bool getCheckApplicationInterfaces() const;
  /*!
   * \brief Sets whether the application interfaces have to be checked.
   *
   * \param check_application interfaces Whether the application interfaces have to be checked.
   */
  void setCheckApplicationInterfaces(bool check_application_interfaces);

  /*!
   * \brief Gets if the device has to be checked.
   *
   * \returns If the device has to be checked.
   */
  bool getCheckDevice() const;
  /*!
   * \brief Sets whether the device has to be checked.
   *
   * \param check_device Whether the device has to be checked.
   */
  void setCheckDevice(bool check_device);

  /*!
   * \brief Gets if the setup procedure has to be run.
   *
   * \returns If the setup procedure has to be run.
   */
  bool getRunSetupProcedure() const;
  /*!
   * \brief Sets whether the setup procedure has to be run.
   *
   * \param run_setup_procedure Whether the setup procedure has to be run.
   */
  void setRunSetupProcedure(bool run_setup_procedure);

  /*!
   * \brief Gets if the firmware has to be checked.
   *
   * \returns If the firmware has to be checked.
   */
  bool getCheckFirmware() const;
  /*!
   * \brief Sets whether the firmware has to be checked.
   *
   * \param check_firmware Whether the firmware has to be checked.
   */
  void setCheckFirmware(bool check_firmware);

  /*!
   * \brief Gets if the user has to wait.
   *
   * \returns If the user has to wait.
   */
  bool getWait() const;
  /*!
   * \brief Sets whether the user has to wait.
   *
   * \param wait Whether the user has to wait.
   */
  void setWait(bool wait);

private:
  bool m_confirm_configuration;
  bool m_check_configuration;
  bool m_check_environment;
  bool m_check_application_interfaces;
  bool m_check_device;
  bool m_run_setup_procedure;
  bool m_check_firmware;
  bool m_wait;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_REQUIREDUSERACTION_H
