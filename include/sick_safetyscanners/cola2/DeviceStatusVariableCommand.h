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
 * \file DeviceStatusVariableCommand.h
 *
 * \author  Jonathan Meyer <jmeyer1292@gmail.com>
 * \date    2019-03-11
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETY_SCANNERS_COLA2_DEVICESTATUSVARIABLECOMMAND_H
#define SICK_SAFETY_SCANNERS_COLA2_DEVICESTATUSVARIABLECOMMAND_H

#include <sick_safetyscanners/cola2/VariableCommand.h>
#include <sick_safetyscanners/data_processing/ParseDeviceStatus.h>
#include <sick_safetyscanners/datastructure/DeviceStatus.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to read the device status registers from the sensor.
 */
class DeviceStatusVariableCommand : public VariableCommand
{
public:
  /*!
   * \brief Typedef to reference the base class.
   */
  typedef sick::cola2::VariableCommand base_class;

  /*!
   * \brief Constructor of the command.
   *
   * Takes the current cola2 session and a reference to the device status variable which will be
   * written on execution.
   *
   * \param session The current cola2 session.
   * \param field_data The device status reference which will be modified on execution.
   */
  DeviceStatusVariableCommand(Cola2Session& session,
                              datastructure::DeviceStatus& device_status);

  /*!
   * \brief Adds the data to the telegram.
   *
   * \param telegram The telegram which will be modified by the data.
   */
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;

  /*!
   * \brief Returns if the command can be executed without a session ID. Will return false for most
   * commands except the commands to establish a connection.
   *
   * \returns If the command needs a session ID to be executed.
   */
  bool canBeExecutedWithoutSessionID() const;

  /*!
   * \brief Processes the return from the sensor.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();


private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;
  std::shared_ptr<sick::data_processing::ParseDeviceStatus> m_device_status_parser_ptr;

  sick::datastructure::DeviceStatus& m_device_status;
};

} // namespace cola2
} // namespace sick


#endif // SICK_SAFETY_SCANNERS_COLA2_DEVICESTATUSVARIABLECOMMAND_H
