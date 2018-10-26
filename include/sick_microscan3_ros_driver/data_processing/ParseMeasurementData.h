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
 * \file parseMeasurementData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEMEASUREMENTDATA_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEMEASUREMENTDATA_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/MeasurementData.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

class ParseMeasurementData
{
public:
  ParseMeasurementData();

  datastructure::MeasurementData parseUDPSequence(const datastructure::PacketBuffer& buffer,
                                                  datastructure::Data& header);

private:
  float m_angle;
  float m_angle_delta;
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  void setDataInMeasurementData(const uint8_t*& data_ptr,
                                datastructure::MeasurementData& measurement_data);
  void setNumberOfBeamsInMeasurementData(const uint8_t*& data_ptr,
                                         datastructure::MeasurementData& measurement_data) const;
  void setStartAngleAndDelta(const datastructure::Data& data);
  void setScanPointsInMeasurementData(const uint8_t*& data_ptr,
                                      datastructure::MeasurementData& measurement_data);
  void addScanPointToMeasurementData(uint16_t offset,
                                     const uint8_t*& data_ptr,
                                     datastructure::MeasurementData& measurement_data) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfMeasurementDataIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

}  // namespace data_processing
}  // namespace sick

#endif
