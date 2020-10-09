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
 * \file VariableCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COLA2_VARIABLECOMMAND_H
#define SICK_SAFETYSCANNERS_COLA2_VARIABLECOMMAND_H

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to read a variable from the sensor.
 */
class VariableCommand : public Command
{
public:
  /*!
   * \brief Constructor of the command.
   *
   * \param session The current cola2 session.
   * \param method_index The index of the variable.
   */
  VariableCommand(Cola2Session& session, const uint16_t& variable_index);

  /*!
   * \brief Adds the data to the telegram.
   *
   * \param telegram The telegram which will be modified by the data.
   * \return Completed telegram
   */
  std::vector<uint8_t> addTelegramData(const std::vector<uint8_t>& telegram) const;

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

  uint16_t getVariableIndex() const;
  void setVariableIndex(const uint16_t& variable_index);

private:
  uint16_t m_variable_index;
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COLA2_VARIABLECOMMAND_H
