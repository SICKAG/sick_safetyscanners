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
 * \file FindMeCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#include <sick_safetyscanners/cola2/FindMeCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

FindMeCommand::FindMeCommand(Cola2Session& session, uint16_t blink_time)
  : MethodCommand(session, 14)
  , m_blink_time(blink_time)
{
}

std::vector<uint8_t> FindMeCommand::addTelegramData(const std::vector<uint8_t>& telegram) const
{
  auto base_output   = base_class::addTelegramData(telegram);
  size_t base_length = base_output.size();
  auto output        = expandTelegram(base_output, 2);

  // Add new values after telegram
  const auto new_data_offset_it = output.begin() + base_length + telegram.size();

  writeDataToDataPtr(new_data_offset_it);

  return output;
}

void FindMeCommand::writeDataToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint16LittleEndian(data_ptr + 0, m_blink_time);
}

bool FindMeCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool FindMeCommand::processReply()
{
  return (!base_class::processReply());
}


} // namespace cola2
} // namespace sick
