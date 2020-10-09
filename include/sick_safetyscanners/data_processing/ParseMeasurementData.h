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

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMEASUREMENTDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMEASUREMENTDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/MeasurementData.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

#include <ros/ros.h>
#include <string>
#include <vector>

namespace sick {
namespace data_processing {

/*!
 * \brief Parser for the measurement data from a udp sequence.
 */
class ParseMeasurementData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseMeasurementData();

  /*!
   * \brief Parses the measurement data if it is enabled.
   *
   * \param buffer The incoming udp sequence.
   * \param data The already parsed data to check if it is enabled.
   *
   * \returns The parsed measurement data.
   */
  datastructure::MeasurementData parseUDPSequence(const datastructure::PacketBuffer& buffer,
                                                  datastructure::Data& data);

private:
  float m_angle;
  float m_angle_delta;
  void setDataInMeasurementData(std::vector<uint8_t>::const_iterator data_ptr,
                                datastructure::MeasurementData& measurement_data);
  void setNumberOfBeamsInMeasurementData(std::vector<uint8_t>::const_iterator data_ptr,
                                         datastructure::MeasurementData& measurement_data) const;
  void setStartAngleAndDelta(const datastructure::Data& data);
  void setScanPointsInMeasurementData(std::vector<uint8_t>::const_iterator data_ptr,
                                      datastructure::MeasurementData& measurement_data);
  void addScanPointToMeasurementData(uint16_t offset,
                                     std::vector<uint8_t>::const_iterator data_ptr,
                                     datastructure::MeasurementData& measurement_data) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfMeasurementDataIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMEASUREMENTDATA_H
