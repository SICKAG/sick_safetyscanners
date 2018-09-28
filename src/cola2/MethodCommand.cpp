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
 * \file MethodCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/cola2/MethodCommand.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

MethodCommand::MethodCommand(Cola2Session& session, uint16_t method_index)
  : Command(session, 0x4D, 0x49) // see cola2 manual 0x4D = 'M' and  0x49 = 'I'
  , m_method_index(method_index)
{
  m_writer_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();
}

void MethodCommand::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  uint16_t prevSize = telegram.size();
  telegram.resize(prevSize + 2);
  uint8_t* data_ptr = telegram.data() + prevSize;
  m_writer_ptr->writeuint16_tLittleEndian(data_ptr, m_method_index, 0);
}

bool MethodCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool MethodCommand::processReply()
{
  if (getCommandType() == 'A' && getCommandMode() == 'I') // should return MA? But does return AI
  {
    std::cout << "Command Method Acknowledged" << std::endl;
    return true;
  }
  else
  {
    std::cout << "Command Method Not Accepted " << std::endl;
    return false;
  }
}

uint16_t MethodCommand::getMethodIndex() const
{
  return m_method_index;
}

void MethodCommand::setMethodIndex(const uint16_t& method_index)
{
  m_method_index = method_index;
}

} // namespace cola2
} // namespace sick
