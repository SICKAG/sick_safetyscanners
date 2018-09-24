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

#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/MeasurementData.h>


namespace sick {
namespace data_processing {

class ParseMeasurementData : public AbstractParseUDPSequence
{
public:
  ParseMeasurementData();

  datastructure::MeasurementData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &header);

private:
  float m_angle;
  float m_angle_delta;
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInMeasurementData(const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
  bool setNumberOfBeamsInMeasurementData(const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
  bool setStartAngleAndDelta(datastructure::Data &data);
  bool setScanPointsInMeasurementData(const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
  bool addScanPointToMeasurementData(UINT16 offset, const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
  bool checkIfPreconditionsAreMet(datastructure::Data &data);
  bool checkIfMeasurementDataIsPublished(datastructure::Data &data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data &data);
};

}
}


