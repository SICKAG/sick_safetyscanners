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
 * \file ParseMeasurementData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/data_processing/ParseMeasurementData.h>

namespace sick {
namespace data_processing {

ParseMeasurementData::ParseMeasurementData()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}

datastructure::MeasurementData
ParseMeasurementData::parseUDPSequence(const datastructure::PacketBuffer& buffer,
                                       datastructure::Data& data)
{
  datastructure::MeasurementData measurement_data;
  if (!checkIfPreconditionsAreMet(data))
  {
    measurement_data.setIsEmpty(true);
    return measurement_data;
  }
  const uint8_t* data_ptr(buffer.getBuffer().data() +
                          data.getDataHeaderPtr()->getMeasurementDataBlockOffset());

  setStartAngleAndDelta(data);
  setDataInMeasurementData(data_ptr, measurement_data);
  return measurement_data;
}

bool ParseMeasurementData::checkIfPreconditionsAreMet(const datastructure::Data& data) const
{
  if (!checkIfMeasurementDataIsPublished(data))
  {
    return false;
  }
  if (!checkIfDataContainsNeededParsedBlocks(data))
  {
    return false;
  }
  return true;
}

bool ParseMeasurementData::checkIfMeasurementDataIsPublished(const datastructure::Data& data) const
{
  if (data.getDataHeaderPtr()->getMeasurementDataBlockOffset() == 0 &&
      data.getDataHeaderPtr()->getMeasurementDataBlockSize() == 0)
  {
    return false;
  }
  return true;
}

bool ParseMeasurementData::checkIfDataContainsNeededParsedBlocks(
  const datastructure::Data& data) const
{
  if (data.getDataHeaderPtr()->isEmpty())
  {
    return false;
  }
  if (data.getDerivedValuesPtr()->isEmpty())
  {
    return false;
  }
  return true;
}


void ParseMeasurementData::setDataInMeasurementData(
  const uint8_t*& data_ptr, datastructure::MeasurementData& measurement_data)
{
  setNumberOfBeamsInMeasurementData(data_ptr, measurement_data);
  setScanPointsInMeasurementData(data_ptr, measurement_data);
}

void ParseMeasurementData::setNumberOfBeamsInMeasurementData(
  const uint8_t*& data_ptr, datastructure::MeasurementData& measurement_data) const
{
  measurement_data.setNumberOfBeams(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 0));
}

void ParseMeasurementData::setStartAngleAndDelta(const datastructure::Data& data)
{
  m_angle       = data.getDerivedValuesPtr()->getStartAngle();
  m_angle_delta = data.getDerivedValuesPtr()->getAngularBeamResolution();
}

void ParseMeasurementData::setScanPointsInMeasurementData(
  const uint8_t*& data_ptr, datastructure::MeasurementData& measurement_data)
{
  for (size_t i = 0; i < measurement_data.getNumberOfBeams(); i++)
  {
    addScanPointToMeasurementData(i, data_ptr, measurement_data);
    m_angle += m_angle_delta;
  }
}

void ParseMeasurementData::addScanPointToMeasurementData(
  const uint16_t offset,
  const uint8_t*& data_ptr,
  datastructure::MeasurementData& measurement_data) const
{
  int16_t distance           = m_reader_ptr->readuint16_tLittleEndian(data_ptr, (4 + offset * 4));
  uint8_t reflectivity       = m_reader_ptr->readuint8_tLittleEndian(data_ptr, (6 + offset * 4));
  uint8_t status             = m_reader_ptr->readuint8_tLittleEndian(data_ptr, (7 + offset * 4));
  bool valid                 = status & (0x01 << 0);
  bool infinite              = status & (0x01 << 1);
  bool glare                 = status & (0x01 << 2);
  bool reflector             = status & (0x01 << 3);
  bool contamination         = status & (0x01 << 4);
  bool contamination_warning = status & (0x01 << 5);
  measurement_data.addScanPoint(sick::datastructure::ScanPoint(m_angle,
                                                               distance,
                                                               reflectivity,
                                                               valid,
                                                               infinite,
                                                               glare,
                                                               reflector,
                                                               contamination,
                                                               contamination_warning));
}

} // namespace data_processing
} // namespace sick
