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
  const std::shared_ptr<std::vector<uint8_t> const> vecPtr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr            = vecPtr->begin();
  bool valid                                               = isValid(data_ptr);
  field_data.setIsValid(valid);

  if (valid)
  {
    setFieldType(data_ptr, field_data);
    uint16_t set_index = readSetIndex(data_ptr);
    field_data.setFieldSetIndex(set_index);
  }
  return true;
}

bool ParseFieldHeaderData::isValid(std::vector<uint8_t>::const_iterator data_ptr) const
{
  bool res     = false;
  uint8_t byte = ReadWriteHelper::readUint8(data_ptr + 0);
  if (byte == 'R' || byte == 'Y')
  {
    res = true;
  }

  return res;
}

void ParseFieldHeaderData::setFieldType(std::vector<uint8_t>::const_iterator data_ptr,
                                        datastructure::FieldData& field_data) const
{
  uint8_t field_type = readFieldType(data_ptr);
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


uint8_t ParseFieldHeaderData::readFieldType(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return ReadWriteHelper::readUint8(data_ptr + 73);
}

uint16_t ParseFieldHeaderData::readSetIndex(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return ReadWriteHelper::readUint16LittleEndian(data_ptr + 82);
}


} // namespace data_processing
} // namespace sick
