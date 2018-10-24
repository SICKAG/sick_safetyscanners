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

#include <sick_microscan3_ros_driver/data_processing/ParseFieldHeaderData.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseFieldHeaderData::ParseFieldHeaderData()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}


bool ParseFieldHeaderData::parseTCPSequence(const datastructure::PacketBuffer buffer,
                                      datastructure::FieldData &field_data)
{
  setFieldType(buffer,field_data);
  return true;
}

void ParseFieldHeaderData::setFieldType(const datastructure::PacketBuffer buffer,
                                      datastructure::FieldData &field_data)
{
  int field_type = readFieldType(buffer);
  field_data.setIsWarningField(false);
  field_data.setIsProtectiveField(false);
  if(field_type == 4 || field_type == 14)
  {
    field_data.setIsProtectiveField(true);
  }
  else if(field_type == 5 || field_type == 15)
  {
    field_data.setIsWarningField(true);
  }

  std::cout << field_type << std::endl;
  std::cout << readSetIndex(buffer) << std::endl;

}



int ParseFieldHeaderData::readFieldType(const datastructure::PacketBuffer buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint8_t(data_ptr, 75);
}

int ParseFieldHeaderData::readSetIndex(const datastructure::PacketBuffer buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint16_tLittleEndian(data_ptr, 84);
}







} // namespace data_processing
} // namespace sick
