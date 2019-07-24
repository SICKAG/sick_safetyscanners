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
 * \file MonitoringCaseTableHeaderVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-17
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/MonitoringCaseTableHeaderVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

// TODO

MonitoringCaseTableHeaderVariableCommand::MonitoringCaseTableHeaderVariableCommand(
  Cola2Session& session, datastructure::FieldData& field_data)
  : VariableCommand(session, 2100)
{
  m_field_header_parser_ptr = std::make_shared<sick::data_processing::ParseFieldHeaderData>();
}

bool MonitoringCaseTableHeaderVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool MonitoringCaseTableHeaderVariableCommand::processReply()
{
  bool result = true;
  if (!base_class::processReply())
  {
    result = false;
  }
  // else
  //{
  // m_field_header_parser_ptr->parseTCPSequence(getDataVector(),m_field_data);
  //}
  return result;
}


} // namespace cola2
} // namespace sick
