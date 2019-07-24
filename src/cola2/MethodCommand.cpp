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
 * \file MethodCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/MethodCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

MethodCommand::MethodCommand(Cola2Session& session, const uint16_t& method_index)
  : Command(session, 0x4D, 0x49) // see cola2 manual 0x4D = 'M' and  0x49 = 'I'
  , m_method_index(method_index)
{
}

std::vector<uint8_t> MethodCommand::addTelegramData(const std::vector<uint8_t>& telegram) const
{
  auto output = expandTelegram(telegram, 2);
  // Add new values after telegram
  auto new_data_offset_it = output.begin() + telegram.size();
  read_write_helper::writeUint16LittleEndian(new_data_offset_it, m_method_index);
  return output;
}

bool MethodCommand::canBeExecutedWithoutSessionID() const
{
  return false;
}

bool MethodCommand::processReply()
{
  bool result = false;
  if ((getCommandType() == 'A' && getCommandMode() == 'I') ||
      (getCommandType() == 0x41 && getCommandMode() == 0x49))
  {
    ROS_INFO("Command Method Acknowledged.");
    result = true;
  }
  else
  {
    ROS_WARN("Command Method Not Accepted.");
  }
  return result;
}

uint16_t MethodCommand::getMethodIndex() const
{
  return m_method_index;
}

void MethodCommand::setMethodIndex(const uint16_t& method_index)
{
  m_method_index = method_index;
}

} // namespace cola2
} // namespace sick
