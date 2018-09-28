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

#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>

#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseGeneralSystemState : public AbstractParseUDPSequence
{
public:
  ParseGeneralSystemState();

  datastructure::GeneralSystemState parseUDPSequence(sick::datastructure::PacketBuffer buffer,
                                                     datastructure::Data& data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInGeneralSystemState(const BYTE* data_ptr,
                                   datastructure::GeneralSystemState& general_System_state);
  bool setStatusBitsInGeneralSystemState(const BYTE* data_ptr,
                                         datastructure::GeneralSystemState& general_System_state);
  bool
  setSafeCutOffPathInGeneralSystemState(const BYTE* data_ptr,
                                        datastructure::GeneralSystemState& general_System_state);
  bool
  setNonSafeCutOffPathInGeneralSystemState(const BYTE* data_ptr,
                                           datastructure::GeneralSystemState& general_System_state);
  bool setResetRequiredCutOffPathInGeneralSystemState(
    const BYTE* data_ptr, datastructure::GeneralSystemState& general_System_state);
  bool setCurrentMonitoringCasesInGeneralSystemState(
    const BYTE* data_ptr, datastructure::GeneralSystemState& general_System_state);
  bool setErrorsInGeneralSystemState(const BYTE* data_ptr,
                                     datastructure::GeneralSystemState& general_System_state);
  bool checkIfPreconditionsAreMet(datastructure::Data& data);
  bool checkIfGeneralSystemStateIsPublished(datastructure::Data& data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data& data);
};

} // namespace data_processing
} // namespace sick
