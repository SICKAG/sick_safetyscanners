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

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEGENERALSYSTEMSTATE_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEGENERALSYSTEMSTATE_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

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
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  void setDataInGeneralSystemState(const uint8_t*& data_ptr,
                                   datastructure::GeneralSystemState& general_System_state) const;
  void
  setStatusBitsInGeneralSystemState(const uint8_t*& data_ptr,
                                    datastructure::GeneralSystemState& general_System_state) const;
  void setSafeCutOffPathInGeneralSystemState(
    const uint8_t*& data_ptr, datastructure::GeneralSystemState& general_System_state) const;
  void setNonSafeCutOffPathInGeneralSystemState(
    const uint8_t*& data_ptr, datastructure::GeneralSystemState& general_System_state) const;
  void setResetRequiredCutOffPathInGeneralSystemState(
    const uint8_t*& data_ptr, datastructure::GeneralSystemState& general_System_state) const;
  void setCurrentMonitoringCasesInGeneralSystemState(
    const uint8_t*& data_ptr, datastructure::GeneralSystemState& general_System_state) const;
  void setErrorsInGeneralSystemState(const uint8_t*& data_ptr,
                                     datastructure::GeneralSystemState& general_System_state) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfGeneralSystemStateIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEGENERALSYSTEMSTATE_H
