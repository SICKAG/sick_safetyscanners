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
 * \file ParseData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEDATA_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEDATA_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ParseApplicationData.h>
#include <sick_microscan3_ros_driver/data_processing/ParseDataHeader.h>
#include <sick_microscan3_ros_driver/data_processing/ParseDerivedValues.h>
#include <sick_microscan3_ros_driver/data_processing/ParseGeneralSystemState.h>
#include <sick_microscan3_ros_driver/data_processing/ParseIntrusionData.h>
#include <sick_microscan3_ros_driver/data_processing/ParseMeasurementData.h>
#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>


namespace sick {
namespace data_processing {

class ParseData
{
public:
  ParseData();

  bool parseUDPSequence(const sick::datastructure::PacketBuffer buffer,
                        sick::datastructure::Data& data) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  std::shared_ptr<sick::data_processing::ParseDataHeader> m_data_header_parser_ptr;
  std::shared_ptr<sick::data_processing::ParseDerivedValues> m_derived_values_parser_ptr;
  std::shared_ptr<sick::data_processing::ParseMeasurementData> m_measurement_data_parser_ptr;
  std::shared_ptr<sick::data_processing::ParseGeneralSystemState> m_general_system_state_parser_ptr;
  std::shared_ptr<sick::data_processing::ParseIntrusionData> m_intrusion_data_parser_ptr;
  std::shared_ptr<sick::data_processing::ParseApplicationData> m_application_data_parser_ptr;


  void setDataBlocksInData(const datastructure::PacketBuffer& buffer,
                           datastructure::Data& data) const;
  void setDataHeaderInData(const datastructure::PacketBuffer& buffer,
                           datastructure::Data& data) const;
  void setDerivedValuesInData(const datastructure::PacketBuffer& buffer,
                              datastructure::Data& data) const;
  void setMeasurementDataInData(const datastructure::PacketBuffer& buffer,
                                datastructure::Data& data) const;
  void setGeneralSystemStateInData(const datastructure::PacketBuffer& buffer,
                                   datastructure::Data& data) const;
  void setIntrusionDataInData(const datastructure::PacketBuffer& buffer,
                              datastructure::Data& data) const;
  void setApplicationDataInData(const datastructure::PacketBuffer& buffer,
                                datastructure::Data& data) const;
};

}  // namespace data_processing
}  // namespace sick

#endif
