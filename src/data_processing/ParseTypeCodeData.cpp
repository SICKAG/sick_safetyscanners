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

#include <sick_safetyscanners/data_processing/ParseTypeCodeData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseTypeCodeData::ParseTypeCodeData() {}


bool ParseTypeCodeData::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                         sick::datastructure::TypeCode& type_code) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  type_code.setTypeCode(readTypeCode(data_ptr));
  type_code.setInterfaceType(readInterfaceType(data_ptr));
  type_code.setMaxRange(readMaxRange(data_ptr));
  return true;
}

std::string ParseTypeCodeData::readTypeCode(std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint16_t code_length = read_write_helper::readUint16LittleEndian(data_ptr);
  std::string code;
  for (uint8_t i = 0; i < code_length; i++)
  {
    code.push_back(read_write_helper::readUint8(data_ptr + 2 + i));
  }
  return code;
}

uint8_t ParseTypeCodeData::readInterfaceType(std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint8_t type_code_interface_1 = read_write_helper::readUint8(data_ptr + 14);
  uint8_t type_code_interface_2 = read_write_helper::readUint8(data_ptr + 15);

  uint8_t res = sick::datastructure::e_interface_type::E_EFIPRO;

  if ((type_code_interface_1 == 'Z' && type_code_interface_2 == 'A') ||
      (type_code_interface_1 == 'A' && type_code_interface_2 == 'A'))
  {
    res = sick::datastructure::e_interface_type::E_EFIPRO;
  }
  else if (type_code_interface_1 == 'I' && type_code_interface_2 == 'Z')
  {
    res = sick::datastructure::e_interface_type::E_ETHERNET_IP;
  }
  else if ((type_code_interface_1 == 'P' && type_code_interface_2 == 'Z') ||
           (type_code_interface_1 == 'L' && type_code_interface_2 == 'Z'))
  {
    res = sick::datastructure::e_interface_type::E_PROFINET;
  }
  else if (type_code_interface_1 == 'A' && type_code_interface_2 == 'N')
  {
    res = sick::datastructure::e_interface_type::E_NONSAFE_ETHERNET;
  }

  return res;
}

float ParseTypeCodeData::readMaxRange(std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint8_t type_code_interface_1 = read_write_helper::readUint8(data_ptr + 12);
  uint8_t type_code_interface_2 = read_write_helper::readUint8(data_ptr + 13);

  int res = sick::datastructure::e_ranges::E_NORMAL_RANGE;

  if ((type_code_interface_1 == '3' && type_code_interface_2 == '0') ||
      (type_code_interface_1 == '4' && type_code_interface_2 == '0') ||
      (type_code_interface_1 == '5' && type_code_interface_2 == '5'))
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
