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
 * \file ParseFieldGeometryData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDGEOMETRYDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDGEOMETRYDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/FieldData.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

#include <vector>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read field geometry data.
 */
class ParseFieldGeometryData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseFieldGeometryData();

  /*!
   * \brief Parses a tcp sequence and return the field geometry data of the warning and protective
   * fields.
   *
   * \param buffer The incoming tcp sequence.
   * \param field_data Reference to the field data to set the geometry data.
   *
   * \returns If parsing the sequence was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::FieldData& field_data) const;

private:
  uint32_t readArrayLength(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readArrayElement(std::vector<uint8_t>::const_iterator data_ptr,
                            uint32_t elem_number) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDGEOMETRYDATA_H
