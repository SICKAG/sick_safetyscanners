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
 * \file MonitoringCaseVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-11-28
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/MonitoringCaseVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {


MonitoringCaseVariableCommand::MonitoringCaseVariableCommand(
  Cola2Session& session,
  datastructure::MonitoringCaseData& monitoring_case_data,
  const uint16_t& index)
  : VariableCommand(session, 2101 + index)
  , m_monitoring_case_data(monitoring_case_data)
{
  m_monitoring_case_parser_ptr = std::make_shared<sick::data_processing::ParseMonitoringCaseData>();
}

bool MonitoringCaseVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool MonitoringCaseVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }
  m_monitoring_case_parser_ptr->parseTCPSequence(getDataVector(), m_monitoring_case_data);
  return true;
}


} // namespace cola2
} // namespace sick
