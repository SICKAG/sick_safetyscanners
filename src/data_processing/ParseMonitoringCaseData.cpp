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
 * \file ParseMonitoringCaseData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-11-29
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseMonitoringCaseData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseMonitoringCaseData::ParseMonitoringCaseData() {}


bool ParseMonitoringCaseData::parseTCPSequence(
  const datastructure::PacketBuffer& buffer,
  sick::datastructure::MonitoringCaseData& monitoring_case_data) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  bool valid                                                = isValid(data_ptr);
  monitoring_case_data.setIsValid(valid);
  if (valid)
  {
    monitoring_case_data.setMonitoringCaseNumber(readMonitoringCaseNumber(data_ptr));

    std::vector<uint16_t> indices;
    std::vector<bool> fields_valid;
    for (uint8_t i = 0; i < 8; i++)
    {
      indices.push_back(readFieldIndex(data_ptr, i));
      fields_valid.push_back(readFieldValid(data_ptr, i));
    }
    monitoring_case_data.setFieldIndices(indices);
    monitoring_case_data.setFieldsValid(fields_valid);
  }
  return true;
}

bool ParseMonitoringCaseData::isValid(std::vector<uint8_t>::const_iterator data_ptr) const
{
  bool res     = false;
  uint8_t byte = read_write_helper::readUint8(data_ptr + 0);
  if (byte == 'R' || byte == 'Y')
  {
    res = true;
  }
  return res;
}

uint16_t ParseMonitoringCaseData::readMonitoringCaseNumber(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 6);
}

uint16_t ParseMonitoringCaseData::readFieldIndex(std::vector<uint8_t>::const_iterator data_ptr,
                                                 const uint8_t& index) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 158 + (index * 4));
}

bool ParseMonitoringCaseData::readFieldValid(std::vector<uint8_t>::const_iterator data_ptr,
                                             const uint8_t& index) const
{
  uint8_t byte = read_write_helper::readUint8(data_ptr + 157 + (index * 4));

  return static_cast<bool>(byte & (0x01 << 0));
}

} // namespace data_processing
} // namespace sick
