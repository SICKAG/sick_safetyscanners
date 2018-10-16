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
 * \file ParseTypeCodeData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/data_processing/ParseTypeCodeData.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseTypeCodeData::ParseTypeCodeData()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}


bool ParseTypeCodeData::parseTCPSequence(datastructure::PacketBuffer buffer,
                                      sick::datastructure::TypeCode& type_code)
{
  type_code.setInterfaceType(readInterfaceType(buffer));
  type_code.setMaxRange(readMaxRange(buffer));
  return true;
}


int ParseTypeCodeData::readInterfaceType(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  uint8_t type_code_interface_1 = m_reader_ptr->readuint8_t(data_ptr, 16);
  uint8_t type_code_interface_2 = m_reader_ptr->readuint8_t(data_ptr, 17);
  
  int res = sick::datastructure::e_interface_type::E_EFIPRO;

  if ((type_code_interface_1 == 'Z' && type_code_interface_2 == 'A')
      || (type_code_interface_1 == 'A' && type_code_interface_2 == 'A'))
  {
    res = sick::datastructure::e_interface_type::E_EFIPRO;
  }
  else if (type_code_interface_1 == 'I' && type_code_interface_2 == 'Z')
  {
    res = sick::datastructure::e_interface_type::E_ETHERNET_IP;
  }
  else if ((type_code_interface_1 == 'P' && type_code_interface_2 == 'Z')
    || (type_code_interface_1 == 'L' && type_code_interface_2 == 'Z'))
  {
    res = sick::datastructure::e_interface_type::E_PROFINET;
  }
  else if (type_code_interface_1 == 'A' && type_code_interface_2 == 'N')
  {
    res = sick::datastructure::e_interface_type::E_NONSAFE_ETHERNET;
  }
  
  return res;
}

float ParseTypeCodeData::readMaxRange(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  uint8_t type_code_interface_1 = m_reader_ptr->readuint8_t(data_ptr, 14);
  uint8_t type_code_interface_2 = m_reader_ptr->readuint8_t(data_ptr, 14);
  
  int res = sick::datastructure::e_ranges::E_NORMAL_RANGE;

  if ((type_code_interface_1 == '3' && type_code_interface_2 == '0')
      || (type_code_interface_1 == '4' && type_code_interface_2 == '0')
      || (type_code_interface_1 == '5' && type_code_interface_2 == '5'))
  {
    res = sick::datastructure::e_ranges::E_NORMAL_RANGE;
  }
  else if (type_code_interface_1 == '9' && type_code_interface_2 == '0')
  {
    res = sick::datastructure::e_ranges::E_LONG_RANGE;
  }

  return (float)res;
}



} // namespace data_processing
} // namespace sick
