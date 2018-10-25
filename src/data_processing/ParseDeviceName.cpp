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
 * \file ParseDeviceName.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/data_processing/ParseDeviceName.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseDeviceName::ParseDeviceName()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}


bool ParseDeviceName::parseTCPSequence(const datastructure::PacketBuffer &buffer,
                                      std::string &device_name) const
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  device_name = readDeviceName(data_ptr);
  return true;
}


std::string ParseDeviceName::readDeviceName(const uint8_t* &data_ptr) const
{
  uint16_t string_length = m_reader_ptr->readuint16_tLittleEndian(data_ptr, 0);

  std::string name;
  for (uint16_t i = 0; i < string_length; i++)
  {
    name.push_back( m_reader_ptr->readuint16_tLittleEndian(data_ptr, 2 + i));
  }
  return name;
}




} // namespace data_processing
} // namespace sick
