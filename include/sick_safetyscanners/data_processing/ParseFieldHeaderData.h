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
 * \file ParseFieldHeaderData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDHEADERDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDHEADERDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/FieldData.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read the field header for protective and warning fields.
 */
class ParseFieldHeaderData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseFieldHeaderData();

  /*!
   * \brief Parses a tcp sequence to read the header for a warning or protective field.
   *
   * \param buffer The incoming tcp sequence.
   * \param field_data Reference to the field data where the information will be set.
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::FieldData& field_data) const;

private:
  bool isValid(std::vector<uint8_t>::const_iterator data_ptr) const;
  void setFieldType(std::vector<uint8_t>::const_iterator data_ptr,
                    datastructure::FieldData& field_data) const;
  uint8_t readFieldType(std::vector<uint8_t>::const_iterator data_ptr) const;
  std::string readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  bool readIsDefined(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readEvalMethod(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readMultiSampling(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readObjectResolution(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readSetIndex(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint32_t readNameLength(std::vector<uint8_t>::const_iterator data_ptr) const;
  std::string readFieldName(std::vector<uint8_t>::const_iterator data_ptr) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEFIELDHEADERDATA_H
