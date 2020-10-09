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
 * \file FieldHeaderVariableCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-17
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COLA2_FIELDHEADERVARIABLECOMMAND_H
#define SICK_SAFETYSCANNERS_COLA2_FIELDHEADERVARIABLECOMMAND_H


#include <sick_safetyscanners/cola2/VariableCommand.h>
#include <sick_safetyscanners/data_processing/ParseFieldHeaderData.h>
#include <sick_safetyscanners/datastructure/CommSettings.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to read the field header variables.
 */
class FieldHeaderVariableCommand : public VariableCommand
{
public:
  /*!
   * \brief Typedef to reference the base class.
   */
  typedef sick::cola2::VariableCommand base_class;

  /*!
   * \brief Constructor of the command.
   *
   * Reads the header of the field variables. Takes the current cola2 session and a reference to the
   * field data, which will be modified during execution. Depending on the sensor up to 128
   * variables can be defined.
   *
   * \param session The current cola2 session.
   * \param field_data Reference to field data, which will be modified on execution.
   * \param index Index of the variable. Can range from [0, 127].
   */
  FieldHeaderVariableCommand(Cola2Session& session,
                             datastructure::FieldData& field_data,
                             const uint16_t& index);

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
  std::shared_ptr<sick::data_processing::ParseFieldHeaderData> m_field_header_parser_ptr;

  sick::datastructure::FieldData& m_field_data;
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COLA2_FIELDHEADERVARIABLECOMMAND_H
