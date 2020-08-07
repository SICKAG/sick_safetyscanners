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
 * \file ParseData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseData.h>

namespace sick {
namespace data_processing {

ParseData::ParseData()
{
  m_data_header_parser_ptr      = std::make_shared<sick::data_processing::ParseDataHeader>();
  m_derived_values_parser_ptr   = std::make_shared<sick::data_processing::ParseDerivedValues>();
  m_measurement_data_parser_ptr = std::make_shared<sick::data_processing::ParseMeasurementData>();
  m_general_system_state_parser_ptr =
    std::make_shared<sick::data_processing::ParseGeneralSystemState>();
  m_intrusion_data_parser_ptr   = std::make_shared<sick::data_processing::ParseIntrusionData>();
  m_application_data_parser_ptr = std::make_shared<sick::data_processing::ParseApplicationData>();
}

sick::datastructure::Data
ParseData::parseUDPSequence(const datastructure::PacketBuffer& buffer) const
{
  sick::datastructure::Data data;
  setDataBlocksInData(buffer, data);
  return data;
}

void ParseData::setDataBlocksInData(const datastructure::PacketBuffer& buffer,
                                    datastructure::Data& data) const
{
  setDataHeaderInData(buffer, data);

  auto dataHeadPtr = data.getDataHeaderPtr();
  uint32_t expected_size =
    dataHeadPtr->getDerivedValuesBlockSize() + dataHeadPtr->getMeasurementDataBlockSize() +
    dataHeadPtr->getGeneralSystemStateBlockSize() + dataHeadPtr->getIntrusionDataBlockSize() +
    dataHeadPtr->getApplicationDataBlockSize();
  uint32_t actual_size = buffer.getLength();
  if (actual_size < expected_size)
  {
    ROS_WARN("Skipping data, sizes do not match, actual size is smaller then expected "
             "size! If this occurs please report with a stacktrace if the driver crashes at some "
             "other place. ");
    ROS_WARN("Expected minimum size: %i", expected_size);
    ROS_WARN("Actual size: %i", actual_size);
    ROS_WARN("Skipping all data for this message.");

    dataHeadPtr->setDerivedValuesBlockSize(0);
    dataHeadPtr->setDerivedValuesBlockOffset(0);
    dataHeadPtr->setMeasurementDataBlockSize(0);
    dataHeadPtr->setMeasurementDataBlockOffset(0);
    dataHeadPtr->setGeneralSystemStateBlockSize(0);
    dataHeadPtr->setGeneralSystemStateBlockOffset(0);
    dataHeadPtr->setIntrusionDataBlockSize(0);
    dataHeadPtr->setIntrusionDataBlockOffset(0);
    dataHeadPtr->setApplicationDataBlockSize(0);
    dataHeadPtr->setApplicationDataBlockOffset(0);
  }

  setDerivedValuesInData(buffer, data);
  setMeasurementDataInData(buffer, data);
  setGeneralSystemStateInData(buffer, data);
  setIntrusionDataInData(buffer, data);
  setApplicationDataInData(buffer, data);
}

void ParseData::setDataHeaderInData(const datastructure::PacketBuffer& buffer,
                                    datastructure::Data& data) const
{
  sick::datastructure::DataHeader data_header =
    m_data_header_parser_ptr->parseUDPSequence(buffer, data);
  data.setDataHeaderPtr(std::make_shared<sick::datastructure::DataHeader>(data_header));
}

void ParseData::setDerivedValuesInData(const datastructure::PacketBuffer& buffer,
                                       datastructure::Data& data) const
{
  sick::datastructure::DerivedValues derived_values =
    m_derived_values_parser_ptr->parseUDPSequence(buffer, data);
  data.setDerivedValuesPtr(std::make_shared<sick::datastructure::DerivedValues>(derived_values));
}

void ParseData::setMeasurementDataInData(const datastructure::PacketBuffer& buffer,
                                         datastructure::Data& data) const
{
  sick::datastructure::MeasurementData measurement_data =
    m_measurement_data_parser_ptr->parseUDPSequence(buffer, data);
  data.setMeasurementDataPtr(
    std::make_shared<sick::datastructure::MeasurementData>(measurement_data));
}

void ParseData::setGeneralSystemStateInData(const datastructure::PacketBuffer& buffer,
                                            datastructure::Data& data) const
{
  sick::datastructure::GeneralSystemState general_system_state =
    m_general_system_state_parser_ptr->parseUDPSequence(buffer, data);
  data.setGeneralSystemStatePtr(
    std::make_shared<sick::datastructure::GeneralSystemState>(general_system_state));
}

void ParseData::setIntrusionDataInData(const datastructure::PacketBuffer& buffer,
                                       datastructure::Data& data) const
{
  sick::datastructure::IntrusionData intrusion_data =
    m_intrusion_data_parser_ptr->parseUDPSequence(buffer, data);
  data.setIntrusionDataPtr(std::make_shared<sick::datastructure::IntrusionData>(intrusion_data));
}

void ParseData::setApplicationDataInData(const datastructure::PacketBuffer& buffer,
                                         datastructure::Data& data) const
{
  sick::datastructure::ApplicationData application_data =
    m_application_data_parser_ptr->parseUDPSequence(buffer, data);
  data.setApplicationDataPtr(
    std::make_shared<sick::datastructure::ApplicationData>(application_data));
}

} // namespace data_processing
} // namespace sick
