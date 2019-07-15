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
 * \file ParseDerivedValues.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDERIVEDVALUES_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDERIVEDVALUES_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/DerivedValues.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

namespace sick {
namespace data_processing {

/*!
 * \brief Parser for the derived values from the udp data packets.
 */
class ParseDerivedValues
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseDerivedValues();

  /*!
   * \brief Parsed the packet buffer and returns the derived values.
   *
   * \param buffer The incoming packet buffer.
   * \param data The already parsed data. Used for checks if the derived values are enabled.
   *
   * \returns The parsed derived values.
   */
  datastructure::DerivedValues parseUDPSequence(const datastructure::PacketBuffer& buffer,
                                                datastructure::Data& data) const;

private:
  void setDataInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                              datastructure::DerivedValues& derived_values) const;
  void setMultiplicationFactorInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                                              datastructure::DerivedValues& derived_values) const;
  void setNumberOfBeamsInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                                       datastructure::DerivedValues& derived_values) const;
  void setScanTimeInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                                  datastructure::DerivedValues& derived_values) const;
  void setStartAngleInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                                    datastructure::DerivedValues& derived_values) const;
  void setAngularBeamResolutionInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                                               datastructure::DerivedValues& derived_values) const;
  void setInterbeamPeriodInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                                         datastructure::DerivedValues& derived_values) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfDerivedValuesIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDERIVEDVALUES_H
