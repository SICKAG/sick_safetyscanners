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
 * \file ParseIntrusionData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEINTRUSIONDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEINTRUSIONDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/DerivedValues.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

#include <vector>

namespace sick {
namespace data_processing {

/*!
 * \brief Parser to read the intrusion data from a udp sequence.
 */
class ParseIntrusionData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseIntrusionData();

  /*!
   * \brief Parse a udp sequence to read the intrusion data if enabled.
   *
   * \param buffer The incoming sequence.
   * \param data The already parsed data to check if intrusion data is enabled.
   *
   * \returns The parsed intrusion data.
   */
  datastructure::IntrusionData parseUDPSequence(const datastructure::PacketBuffer& buffer,
                                                datastructure::Data& data);
  uint16_t getNumScanPoints() const;
  void setNumScanPoints(const uint16_t& num_scan_points);

private:
  uint16_t m_num_scan_points;

  void setDataInIntrusionData(std::vector<uint8_t>::const_iterator data_ptr,
                              datastructure::IntrusionData& intrusion_data) const;
  void setDataInIntrusionDatums(
    std::vector<uint8_t>::const_iterator data_ptr,
    std::vector<sick::datastructure::IntrusionDatum>& intrusion_datums) const;
  uint16_t setSizeInIntrusionDatum(const uint16_t& offset,
                                   std::vector<uint8_t>::const_iterator data_ptr,
                                   sick::datastructure::IntrusionDatum& datum) const;
  uint16_t setFlagsInIntrusionDatum(const uint16_t& offset,
                                    std::vector<uint8_t>::const_iterator data_ptr,
                                    sick::datastructure::IntrusionDatum& datum) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfIntrusionDataIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEINTRUSIONDATA_H
