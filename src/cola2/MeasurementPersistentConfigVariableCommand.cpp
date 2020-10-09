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
 * \file MeasurementPersistentConfigVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-01-07
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/MeasurementPersistentConfigVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

MeasurementPersistentConfigVariableCommand::MeasurementPersistentConfigVariableCommand(
  Cola2Session& session, datastructure::ConfigData& config_data)
  : VariableCommand(session, 177)
  , m_config_data(config_data)
{
  m_measurement_persistent_config_parser_ptr =
    std::make_shared<sick::data_processing::ParseMeasurementPersistentConfigData>();
}

bool MeasurementPersistentConfigVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool MeasurementPersistentConfigVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }
  m_measurement_persistent_config_parser_ptr->parseTCPSequence(getDataVector(), m_config_data);
  return true;
}


} // namespace cola2
} // namespace sick
