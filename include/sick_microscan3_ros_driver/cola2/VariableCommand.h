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

#ifndef VARIABLECOMMAND_H
#define VARIABLECOMMAND_H

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

class VariableCommand : public Command
{
public:
  VariableCommand(Cola2Session& session, const uint16_t &method_index);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();

  uint16_t getVariableIndex() const;
  void setVariableIndex(const uint16_t& variable_index);

private:
  uint16_t m_variable_index;
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;
};

} // namespace cola2
} // namespace sick

#endif
