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

#include <sick_microscan3_ros_driver/data_processing/ParseFieldGeometryData.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseFieldGeometryData::ParseFieldGeometryData()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}


bool ParseFieldGeometryData::parseTCPSequence(const datastructure::PacketBuffer &buffer,
                                      sick::datastructure::FieldData& field_data) const
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  uint32_t array_length = readArrayLength(data_ptr);
  std::vector<uint16_t> geometry_distance_mm;
  for (uint32_t i = 0 ; i < array_length; i++)
  {
    geometry_distance_mm.push_back(readArrayElement(data_ptr, i));
  }
  field_data.setBeamDistances(geometry_distance_mm);

  return true;
}

uint32_t ParseFieldGeometryData::readArrayLength(const uint8_t *&data_ptr) const
{
  return m_reader_ptr->readuint32_tLittleEndian(data_ptr, 4);
}

uint16_t ParseFieldGeometryData::readArrayElement(const uint8_t *&data_ptr, uint32_t elem_number) const
{
  return m_reader_ptr->readuint16_tLittleEndian(data_ptr, 8 + elem_number * 2);
}



} // namespace data_processing
} // namespace sick
