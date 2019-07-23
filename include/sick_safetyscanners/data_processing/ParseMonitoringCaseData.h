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
 * \file ParseMonitoringCaseData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-11-29
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMONITORINGCASEDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMONITORINGCASEDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/MonitoringCaseData.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

#include <vector>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read monitoring case data.
 */
class ParseMonitoringCaseData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseMonitoringCaseData();

  /*!
   * \brief Parses a tcp sequence and return the monitoring case data.
   *
   * \param buffer The incoming tcp sequence.
   * \param monitoring_case_data Reference to the monitoring case data.
   *
   * \returns If parsing the sequence was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::MonitoringCaseData& monitoring_case_data) const;

private:
  bool isValid(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readMonitoringCaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readFieldIndex(std::vector<uint8_t>::const_iterator data_ptr,
                          const uint8_t& index) const;
  bool readFieldValid(std::vector<uint8_t>::const_iterator data_ptr, const uint8_t& index) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDGEOMETRYDATA_H
