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
 * \file ParseGeneralSystemState.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEGENERALSYSTEMSTATE_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEGENERALSYSTEMSTATE_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/DerivedValues.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

#include <vector>

namespace sick {
namespace data_processing {

/*!
 * \brief Parser to parse the general system state from the udp packets.
 */
class ParseGeneralSystemState
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseGeneralSystemState();

  /*!
   * \brief Parses the udp sequence to read the general system state if enabled.
   *
   * \param buffer The incoming sequence.
   * \param data The already parsed data. Used to check if the general system state is enabled.
   *
   * \returns If parsing was successful.
   */
  datastructure::GeneralSystemState
  parseUDPSequence(const sick::datastructure::PacketBuffer& buffer,
                   datastructure::Data& data) const;

private:
  void setDataInGeneralSystemState(std::vector<uint8_t>::const_iterator data_ptr,
                                   datastructure::GeneralSystemState& general_system_state) const;
  void
  setStatusBitsInGeneralSystemState(std::vector<uint8_t>::const_iterator data_ptr,
                                    datastructure::GeneralSystemState& general_system_state) const;
  void setSafeCutOffPathInGeneralSystemState(
    std::vector<uint8_t>::const_iterator data_ptr,
    datastructure::GeneralSystemState& general_system_state) const;
  void setNonSafeCutOffPathInGeneralSystemState(
    std::vector<uint8_t>::const_iterator data_ptr,
    datastructure::GeneralSystemState& general_system_state) const;
  void setResetRequiredCutOffPathInGeneralSystemState(
    std::vector<uint8_t>::const_iterator data_ptr,
    datastructure::GeneralSystemState& general_system_state) const;
  void setCurrentMonitoringCasesInGeneralSystemState(
    std::vector<uint8_t>::const_iterator data_ptr,
    datastructure::GeneralSystemState& general_system_state) const;
  void setErrorsInGeneralSystemState(std::vector<uint8_t>::const_iterator data_ptr,
                                     datastructure::GeneralSystemState& general_system_state) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfGeneralSystemStateIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEGENERALSYSTEMSTATE_H
