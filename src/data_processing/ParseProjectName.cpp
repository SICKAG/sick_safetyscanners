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
 * \file ParseProjectName.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-23
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseProjectName.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseProjectName::ParseProjectName() {}


bool ParseProjectName::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                        datastructure::ProjectName& project_name) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  project_name.setProjectName(readProjectName(data_ptr));
  return true;
}


std::string ParseProjectName::readProjectName(std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint16_t string_length = read_write_helper::readUint16LittleEndian(data_ptr + 0);

  std::string name;
  for (uint16_t i = 0; i < string_length; i++)
  {
    name.push_back(read_write_helper::readUint8LittleEndian(data_ptr + 2 + i));
  }
  return name;
}


} // namespace data_processing
} // namespace sick
