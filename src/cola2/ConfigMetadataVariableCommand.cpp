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
 * \file ConfigMetadataVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/cola2/ConfigMetadataVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

ConfigMetadataVariableCommand::ConfigMetadataVariableCommand(
  Cola2Session& session, sick::datastructure::ConfigMetadata& config_metadata)
  : VariableCommand(session, 28)
  , m_config_metadata(config_metadata)
{
  m_config_metadata_parser_ptr = std::make_shared<sick::data_processing::ParseConfigMetadata>();
}

bool ConfigMetadataVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool ConfigMetadataVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }
  m_config_metadata_parser_ptr->parseTCPSequence(getDataVector(), m_config_metadata);
  return true;
}


} // namespace cola2
} // namespace sick
