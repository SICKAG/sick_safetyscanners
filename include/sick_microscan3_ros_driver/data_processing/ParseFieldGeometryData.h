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
 * \file ParseFieldGeometryData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEFIELDGEOMETRYDATA_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEFIELDGEOMETRYDATA_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/FieldData.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

#include <vector>

namespace sick {

namespace data_processing {


  /*!
   * \brief Parser to read field geometry data.
   */
class ParseFieldGeometryData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseFieldGeometryData();

  /*!
   * \brief Parses a tcp sequence and return the field geometry data of the warning and protective
   * fields.
   *
   * \param buffer The incoming tcp sequence.
   * \param field_data Reference to the field data to set the geometry data.
   *
   * \returns If parsing the sequence was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::FieldData& field_data) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  uint32_t readArrayLength(const uint8_t*& data_ptr) const;
  uint16_t readArrayElement(const uint8_t*& data_ptr, uint32_t elem_number) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEFIELDGEOMETRYDATA_H
