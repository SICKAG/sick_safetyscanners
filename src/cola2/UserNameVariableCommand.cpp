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
 * \file UserNameVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/UserNameVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

UserNameVariableCommand::UserNameVariableCommand(Cola2Session& session,
                                                 sick::datastructure::UserName& user_name)
  : VariableCommand(session, 35)
  , m_user_name(user_name)
{
  m_user_name_parser_ptr = std::make_shared<sick::data_processing::ParseUserNameData>();
}

bool UserNameVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool UserNameVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }
  m_user_name_parser_ptr->parseTCPSequence(getDataVector(), m_user_name);
  return true;
}


} // namespace cola2
} // namespace sick
