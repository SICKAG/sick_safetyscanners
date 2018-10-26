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
 * \file ChangeCommSettingsCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#ifndef SICK_MICROSCAN3_ROS_DRIVER_COLA2_CHANGECOMMSETTINGSCOMMAND_H
#define SICK_MICROSCAN3_ROS_DRIVER_COLA2_CHANGECOMMSETTINGSCOMMAND_H


#include <sick_microscan3_ros_driver/cola2/MethodCommand.h>
#include <sick_microscan3_ros_driver/datastructure/CommSettings.h>

namespace sick {
namespace cola2 {

class ChangeCommSettingsCommand : public MethodCommand
{
public:
  typedef sick::cola2::MethodCommand base_class;

  ChangeCommSettingsCommand(Cola2Session& session, const datastructure::CommSettings& settings);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();


private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;
  sick::datastructure::CommSettings m_settings;

  uint8_t*
  prepareTelegramAndGetDataPtr(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  void writeDataToDataPtr(uint8_t*& data_ptr) const;
  void writeChannelToDataPtr(uint8_t*& data_ptr) const;
  void writeEnabledToDataPtr(uint8_t*& data_ptr) const;
  void writeEInterfaceTypeToDataPtr(uint8_t*& data_ptr) const;
  void writeIPAdresstoDataPtr(uint8_t*& data_ptr) const;
  void writePortToDataPtr(uint8_t*& data_ptr) const;
  void writeFrequencyToDataPtr(uint8_t*& data_ptr) const;
  void writeStartAngleToDataPtr(uint8_t*& data_ptr) const;
  void writeEndAngleToDataPtr(uint8_t*& data_ptr) const;
  void writeFeaturesToDataPtr(uint8_t*& data_ptr) const;
};

} // namespace cola2
} // namespace sick

#endif // SICK_MICROSCAN3_ROS_DRIVER_COLA2_CHANGECOMMSETTINGSCOMMAND_H
