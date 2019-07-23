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
 * \file CreateSession.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/CreateSession.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

CreateSession::CreateSession(Cola2Session& session)
  : Command(session, 0x4F, 0x58) // see cola2 manual 0x4F = O, 0x58 = X
{
}

std::vector<uint8_t> CreateSession::addTelegramData(const std::vector<uint8_t>& telegram) const
{
  auto output = expandTelegram(telegram, 5);
  // Add new values after telegram
  auto new_data_offset_it = output.begin() + telegram.size();
  writeHeartbeatTimeoutToDataPtr(new_data_offset_it);
  writeClientIdToDataPtr(new_data_offset_it);
  return output;
}

bool CreateSession::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool CreateSession::processReply()
{
  bool result = false;
  if ((getCommandType() == 'O' && getCommandMode() == 'A') ||
      (getCommandType() == 0x4F && getCommandMode() == 0x41))
  {
    m_session.setSessionID(getSessionID());
    ROS_INFO("Successfully opened Cola2 session with sessionID: %u", m_session.getSessionID());
    result = true;
  }
  else
  {
    ROS_WARN("Could not open Cola2 session");
  }
  return result;
}

void CreateSession::writeHeartbeatTimeoutToDataPtr(std::vector<uint8_t>::iterator it) const
{
  uint8_t heart_beat_time_out_seconds = 60;
  read_write_helper::writeUint8BigEndian(it + 0, heart_beat_time_out_seconds);
}

void CreateSession::writeClientIdToDataPtr(std::vector<uint8_t>::iterator it) const
{
  uint32_t client_id = 1; // can be any random number
  read_write_helper::writeUint32BigEndian(it + 1, client_id);
}

} // namespace cola2
} // namespace sick
