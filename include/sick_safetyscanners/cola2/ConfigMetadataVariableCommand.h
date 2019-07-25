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
 * \file ConfigMetadataVariableCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COLA2_CONFIGMETADATAVARIABLECOMMAND_H
#define SICK_SAFETYSCANNERS_COLA2_CONFIGMETADATAVARIABLECOMMAND_H


#include <sick_safetyscanners/cola2/VariableCommand.h>
#include <sick_safetyscanners/data_processing/ParseConfigMetadata.h>
#include <sick_safetyscanners/datastructure/CommSettings.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to read the type code of the sensor.
 */
class ConfigMetadataVariableCommand : public VariableCommand
{
public:
  /*!
   * \brief Typedef to reference the base class.
   */
  typedef sick::cola2::VariableCommand base_class;

  /*!
   * \brief Constructor of the Command.
   *
   * \param session The current cola2 session.
   * \param config_metadata Reference to the config metadata variable, which will be written on
   * execution.
   */
  ConfigMetadataVariableCommand(Cola2Session& session,
                                datastructure::ConfigMetadata& config_metadata);

  /*!
   * \brief Returns if the command can be executed without a session ID. Will return false for most
   * commands except the commands to establish a connection.
   *
   * \returns If the command needs a session ID to be executed.
   */
  bool canBeExecutedWithoutSessionID() const;

  /*!
   * \brief Processes the return from the sensor.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();


private:
  std::shared_ptr<sick::data_processing::ParseConfigMetadata> m_config_metadata_parser_ptr;

  sick::datastructure::ConfigMetadata& m_config_metadata;
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COLA2_CONFIGMETADATAVARIABLECOMMAND_H
