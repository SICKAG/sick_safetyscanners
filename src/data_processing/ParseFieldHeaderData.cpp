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
 * \file ParseFieldHeaderData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseFieldHeaderData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseFieldHeaderData::ParseFieldHeaderData() {}


bool ParseFieldHeaderData::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                            datastructure::FieldData& field_data) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  bool valid                                                = isValid(data_ptr);
  field_data.setIsValid(valid);

  if (valid)
  {
    setFieldType(data_ptr, field_data);
    uint16_t set_index = readSetIndex(data_ptr);
    field_data.setFieldSetIndex(set_index);
    field_data.setVersionCVersion(readVersionIndicator(data_ptr));
    field_data.setVersionMajorVersionNumber(readMajorNumber(data_ptr));
    field_data.setVersionMinorVersionNumber(readMinorNumber(data_ptr));
    field_data.setVersionReleaseNumber(readReleaseNumber(data_ptr));
    field_data.setIsDefined(readIsDefined(data_ptr));
    field_data.setEvalMethod(readEvalMethod(data_ptr));
    field_data.setMultiSampling(readMultiSampling(data_ptr));
    field_data.setObjectResolution(readObjectResolution(data_ptr));
    field_data.setNameLength(readNameLength(data_ptr));
    field_data.setFieldName(readFieldName(data_ptr));
  }
  return true;
}

bool ParseFieldHeaderData::isValid(std::vector<uint8_t>::const_iterator data_ptr) const
{
  bool res     = false;
  uint8_t byte = read_write_helper::readUint8(data_ptr + 0);
  if (byte == 'R' || byte == 'Y')
  {
    res = true;
  }

  return res;
}

void ParseFieldHeaderData::setFieldType(std::vector<uint8_t>::const_iterator data_ptr,
                                        datastructure::FieldData& field_data) const
{
  uint8_t field_type = readEvalMethod(data_ptr);
  field_data.setIsWarningField(false);
  field_data.setIsProtectiveField(false);
  if (field_type == 4 || field_type == 14)
  {
    field_data.setIsProtectiveField(true);
  }
  else if (field_type == 5 || field_type == 15)
  {
    field_data.setIsWarningField(true);
  }
}

std::string
ParseFieldHeaderData::readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::string result;
  result.push_back(read_write_helper::readUint8(data_ptr + 0));
  return result;
}

uint8_t ParseFieldHeaderData::readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 1);
}

uint8_t ParseFieldHeaderData::readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 2);
}

uint8_t ParseFieldHeaderData::readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 3);
}

bool ParseFieldHeaderData::readIsDefined(std::vector<uint8_t>::const_iterator data_ptr) const
{
  // TODO
  return read_write_helper::readUint8(data_ptr + 72);
}

uint8_t ParseFieldHeaderData::readEvalMethod(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 73);
}


uint16_t
ParseFieldHeaderData::readMultiSampling(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 74);
}

uint16_t
ParseFieldHeaderData::readObjectResolution(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 78);
}

uint16_t ParseFieldHeaderData::readSetIndex(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 82);
}


uint32_t ParseFieldHeaderData::readNameLength(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 84);
}


std::string ParseFieldHeaderData::readFieldName(std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint32_t name_length = read_write_helper::readUint32LittleEndian(data_ptr + 84);
  std::string name;
  for (uint8_t i = 0; i < name_length; i++)
  {
    name.push_back(read_write_helper::readUint8(data_ptr + 88 + i));
  }
  return name;
}
} // namespace data_processing
} // namespace sick
