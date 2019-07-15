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
 * \file TypeCodeVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/TypeCodeVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

TypeCodeVariableCommand::TypeCodeVariableCommand(Cola2Session& session,
                                                 sick::datastructure::TypeCode& type_code)
  : VariableCommand(session, 0x000d)
  , m_type_code(type_code)
{
  m_type_code_parser_ptr = std::make_shared<sick::data_processing::ParseTypeCodeData>();
}

bool TypeCodeVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool TypeCodeVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }
  m_type_code_parser_ptr->parseTCPSequence(getDataVector(), m_type_code);
  return true;
}


} // namespace cola2
} // namespace sick
