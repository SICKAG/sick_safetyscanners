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
 * \file FirmwareVersionVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019_07_23
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/FirmwareVersionVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

FirmwareVersionVariableCommand::FirmwareVersionVariableCommand(
  Cola2Session& session, datastructure::FirmwareVersion& firmware_version)
  : VariableCommand(session, 4)
  , m_firmware_version(firmware_version)
{
  m_firmware_version_parser_ptr = std::make_shared<sick::data_processing::ParseFirmwareVersion>();
}

bool FirmwareVersionVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool FirmwareVersionVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }

  m_firmware_version_parser_ptr->parseTCPSequence(getDataVector(), m_firmware_version);


  return true;
}


} // namespace cola2
} // namespace sick
