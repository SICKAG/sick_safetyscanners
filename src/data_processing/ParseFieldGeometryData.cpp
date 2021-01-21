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
 * \file ParseFieldGeometryData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseFieldGeometryData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseFieldGeometryData::ParseFieldGeometryData() {}


bool ParseFieldGeometryData::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                              sick::datastructure::FieldData& field_data) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  uint32_t array_length                                     = readArrayLength(data_ptr);
  std::vector<uint16_t> geometry_distance_mm;
  for (uint32_t i = 0; i < array_length; i++)
  {
    geometry_distance_mm.push_back(readArrayElement(data_ptr, i));
  }
  field_data.setBeamDistances(geometry_distance_mm);

  // Values are persistent for all scanners
  field_data.setStartAngleDegrees(-47.5); // defined start angle in degrees in sick coordinates
  float res = 275.0 / array_length;       // 275 is the defined field of view
  field_data.setAngularBeamResolutionDegrees(res);

  return true;
}

uint32_t
ParseFieldGeometryData::readArrayLength(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 4);
}

uint16_t ParseFieldGeometryData::readArrayElement(std::vector<uint8_t>::const_iterator data_ptr,
                                                  uint32_t elem_number) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 8 + elem_number * 2);
}


} // namespace data_processing
} // namespace sick
