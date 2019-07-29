// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2019, SICK AG, Waldkirch
*  Copyright (C) 2019, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file ParseStatusOverview.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-22
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSESTATUSOVERVIEW_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSESTATUSOVERVIEW_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>
#include <sick_safetyscanners/datastructure/StatusOverview.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read the status overview of a tcp sequence.
 */
class ParseStatusOverviewData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseStatusOverviewData();

  /*!
   * \brief Parses a tcp sequence to read the status overview of the sensor.
   *
   * \param buffer The incoming tcp sequence.
   * \param status_overview Reference to the status overview, which will be written while parsing.
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::StatusOverview& status_overview) const;

private:
  std::string readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readDeviceState(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readConfigState(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readApplicationState(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint32_t readPowerOnCount(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint32_t readCurrentTime(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readCurrentDate(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint32_t readErrorInfoCode(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint32_t readErrorInfoTime(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readErrorInfoDate(std::vector<uint8_t>::const_iterator data_ptr) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSESTATUSOVERVIEW_H
