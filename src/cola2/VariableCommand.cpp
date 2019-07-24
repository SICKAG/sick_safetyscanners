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
 * \file VariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/VariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

VariableCommand::VariableCommand(Cola2Session& session, const uint16_t& variable_index)
  : Command(session, 0x52, 0x49) // see cola2 manual 0x52 = 'R' and  0x49 = 'I'
  , m_variable_index(variable_index)
{
}

std::vector<uint8_t> VariableCommand::addTelegramData(const std::vector<uint8_t>& telegram) const
{
  auto output = expandTelegram(telegram, 2);
  // Add new values after telegram
  auto new_data_offset_it = output.begin() + telegram.size();
  read_write_helper::writeUint16LittleEndian(new_data_offset_it, m_variable_index);
  return output;
}

bool VariableCommand::canBeExecutedWithoutSessionID() const
{
  return false;
}

bool VariableCommand::processReply()
{
  bool result = false;
  if ((getCommandType() == 'R' && getCommandMode() == 'A') ||
      (getCommandType() == 0x52 && getCommandMode() == 0x41))
  {
    ROS_INFO("Command Variable Acknowledged.");
    result = true;
  }
  else
  {
    ROS_WARN("Command Variable Not Accepted.");
    result = false;
  }
  return result;
}

uint16_t VariableCommand::getVariableIndex() const
{
  return m_variable_index;
}

void VariableCommand::setVariableIndex(const uint16_t& variable_index)
{
  m_variable_index = variable_index;
}

} // namespace cola2
} // namespace sick
