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
 * \file MethodCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#pragma once

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

class MethodCommand : public Command
{
public:
  MethodCommand(Cola2Session& session, UINT16 method_index);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();

  UINT16 getMethodIndex() const;
  void setMethodIndex(const UINT16& method_index);

private:
  UINT16 m_method_index;
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;
};

} // namespace cola2
} // namespace sick
