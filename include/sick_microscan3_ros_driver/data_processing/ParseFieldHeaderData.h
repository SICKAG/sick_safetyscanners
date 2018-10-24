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
 * \file ParseFieldHeaderData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef PARSEFIELDHEADERDATA_H
#define PARSEFIELDHEADERDATA_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/datastructure/FieldData.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {

namespace cola2 {
class Command;
}

namespace data_processing {


class ParseFieldHeaderData
{
public:
  ParseFieldHeaderData();

  bool parseTCPSequence(datastructure::PacketBuffer buffer, datastructure::FieldData &field_data);

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  void setFieldType(datastructure::PacketBuffer buffer, datastructure::FieldData &field_data);
  int readFieldType(datastructure::PacketBuffer buffer);
  int readSetIndex(const datastructure::PacketBuffer buffer);
};

} // namespace data_processing
} // namespace sick

#endif
