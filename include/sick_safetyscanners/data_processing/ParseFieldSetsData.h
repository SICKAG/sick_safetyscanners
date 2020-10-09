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
 * \file ParseFieldSetsData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-22
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDSETSDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDSETSDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/FieldSets.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read the type code of a tcp sequence.
 */
class ParseFieldSetsData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseFieldSetsData();

  /*!
   * \brief Parses a tcp sequence to read the field sets of the sensor.
   *
   * \param buffer The incoming tcp sequence.
   * \param field_sets Reference to the field sets, which will be written while parsing.
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::FieldSets& field_sets) const;

private:
  std::string readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint32_t readArrayLength(std::vector<uint8_t>::const_iterator data_ptr) const;
  std::vector<uint32_t> readNameLength(std::vector<uint8_t>::const_iterator data_ptr,
                                       uint32_t array_length) const;
  std::vector<std::string> readFieldName(std::vector<uint8_t>::const_iterator data_ptr,
                                         uint32_t array_length) const;
  std::vector<bool> readIsDefined(std::vector<uint8_t>::const_iterator data_ptr,
                                  uint32_t array_length) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDSETSDATA_H
