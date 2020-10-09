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

#include <sick_safetyscanners/datastructure/RequiredUserAction.h>

namespace sick {
namespace datastructure {

RequiredUserAction::RequiredUserAction() {}

bool RequiredUserAction::getConfirmConfiguration() const
{
  return m_confirm_configuration;
}

void RequiredUserAction::setConfirmConfiguration(bool confirm_configuration)
{
  m_confirm_configuration = confirm_configuration;
}

bool RequiredUserAction::getCheckConfiguration() const
{
  return m_check_configuration;
}

void RequiredUserAction::setCheckConfiguration(bool check_configuration)
{
  m_check_configuration = check_configuration;
}

bool RequiredUserAction::getCheckEnvironment() const
{
  return m_check_environment;
}

void RequiredUserAction::setCheckEnvironment(bool check_environment)
{
  m_check_environment = check_environment;
}

bool RequiredUserAction::getCheckApplicationInterfaces() const
{
  return m_check_application_interfaces;
}

void RequiredUserAction::setCheckApplicationInterfaces(bool check_application_interfaces)
{
  m_check_application_interfaces = check_application_interfaces;
}

bool RequiredUserAction::getCheckDevice() const
{
  return m_check_device;
}

void RequiredUserAction::setCheckDevice(bool check_device)
{
  m_check_device = check_device;
}

bool RequiredUserAction::getRunSetupProcedure() const
{
  return m_run_setup_procedure;
}

void RequiredUserAction::setRunSetupProcedure(bool run_setup_procedure)
{
  m_run_setup_procedure = run_setup_procedure;
}

bool RequiredUserAction::getCheckFirmware() const
{
  return m_check_firmware;
}

void RequiredUserAction::setCheckFirmware(bool check_firmware)
{
  m_check_firmware = check_firmware;
}

bool RequiredUserAction::getWait() const
{
  return m_wait;
}

void RequiredUserAction::setWait(bool wait)
{
  m_wait = wait;
}

} // namespace datastructure
} // namespace sick
