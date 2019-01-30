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

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ParseApplicationData.h>
#include <sick_safetyscanners/data_processing/ParseDataHeader.h>
#include <sick_safetyscanners/data_processing/ParseDerivedValues.h>
#include <sick_safetyscanners/data_processing/ParseGeneralSystemState.h>
#include <sick_safetyscanners/data_processing/ParseIntrusionData.h>
#include <sick_safetyscanners/data_processing/ParseMeasurementData.h>
#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>


namespace sick {
namespace data_processing {

/*!
 * \brief Parses the udp data packets depending on which data will be received.
 */
class ParseData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseData();

  /*!
   * \brief Parses the udp data transferred in the packet buffer. It will be parsed into the data
   * reference.
   *
   * \param buffer The incoming data buffer.
   *
   * \returns Parsed data
   */
  sick::datastructure::Data parseUDPSequence(const sick::datastructure::PacketBuffer& buffer) const;

private:
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

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDATA_H
