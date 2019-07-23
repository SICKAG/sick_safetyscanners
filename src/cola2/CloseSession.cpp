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
 * \file CloseSession.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#include <sick_safetyscanners/cola2/CloseSession.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

CloseSession::CloseSession(Cola2Session& session)
  : Command(session, 0x43, 0x58) // see cola2 manual 0x43 = C, 0x58 = X
{
}

std::vector<uint8_t> CloseSession::addTelegramData(const std::vector<uint8_t>& telegram) const
{
  return telegram;
}

bool CloseSession::canBeExecutedWithoutSessionID() const
{
  return false;
}

bool CloseSession::processReply()
{
  bool result = false;
  if ((getCommandType() == 'C' && getCommandMode() == 'A') ||
      (getCommandType() == 0x43 && getCommandMode() == 0x41))
  {
    m_session.setSessionID(getSessionID());
    ROS_INFO("Successfully closed Cola2 session with sessionID: %u", m_session.getSessionID());
    result = true;
  }
  else
  {
    ROS_WARN("Could not close Cola2 session with sessionID: %u", m_session.getSessionID());
  }
  return result;
}

} // namespace cola2
} // namespace sick
