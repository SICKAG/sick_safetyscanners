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
 * \file ParseFieldSetsData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-22
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseFieldSetsData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseFieldSetsData::ParseFieldSetsData() {}


bool ParseFieldSetsData::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                          sick::datastructure::FieldSets& field_sets) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  field_sets.setVersionCVersion(readVersionIndicator(data_ptr));
  field_sets.setVersionMajorVersionNumber(readMajorNumber(data_ptr));
  field_sets.setVersionMinorVersionNumber(readMinorNumber(data_ptr));
  field_sets.setVersionReleaseNumber(readReleaseNumber(data_ptr));
  uint32_t array_length = readArrayLength(data_ptr);
  field_sets.setNameLength(readNameLength(data_ptr, array_length));
  field_sets.setFieldName(readFieldName(data_ptr, array_length));
  field_sets.setIsDefined(readIsDefined(data_ptr, array_length));
  return true;
}

std::string
ParseFieldSetsData::readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::string result;
  result.push_back(read_write_helper::readUint8(data_ptr + 0));
  return result;
}

uint8_t ParseFieldSetsData::readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 1);
}

uint8_t ParseFieldSetsData::readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 2);
}

uint8_t ParseFieldSetsData::readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 3);
}

uint32_t ParseFieldSetsData::readArrayLength(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 4);
}


std::vector<std::string>
ParseFieldSetsData::readFieldName(std::vector<uint8_t>::const_iterator data_ptr,
                                  uint32_t array_length) const
{
  std::vector<std::string> result;
  for (uint32_t i = 0; i < array_length; i++)
  {
    uint32_t name_length = read_write_helper::readUint32LittleEndian(data_ptr + 8 + i * 104);
    std::string name;
    for (uint8_t j = 0; j < name_length; j++)
    {
      name.push_back(read_write_helper::readUint8(data_ptr + 12 + i * 104 + j));
    }
    result.push_back(name);
  }
  return result;
}

std::vector<uint32_t>
ParseFieldSetsData::readNameLength(std::vector<uint8_t>::const_iterator data_ptr,
                                   uint32_t array_length) const
{
  std::vector<uint32_t> result;
  for (uint32_t i = 0; i < array_length; i++)
  {
    result.push_back(read_write_helper::readUint32LittleEndian(data_ptr + 8 + i * 104));
  }
  return result;
}
std::vector<bool> ParseFieldSetsData::readIsDefined(std::vector<uint8_t>::const_iterator data_ptr,
                                                    uint32_t array_length) const
{
  std::vector<bool> result;
  for (uint32_t i = 0; i < array_length; i++)
  {
    uint8_t byte = read_write_helper::readUint8LittleEndian(data_ptr + 44 + i * 104);

    result.push_back(static_cast<bool>(byte & (0x01 << 0)));
  }
  return result;
}
} // namespace data_processing
} // namespace sick
