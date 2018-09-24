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

#pragma once

#include <sick_microscan3_ros_driver/cola2/MethodCommand.h>
#include <sick_microscan3_ros_driver/datastructure/CommSettings.h>

namespace sick {
namespace cola2 {

class ChangeCommSettingsCommand : public MethodCommand
{
public:
  typedef sick::cola2::MethodCommand base_class;

  ChangeCommSettingsCommand(Cola2Session& session, sick::datastructure::CommSettings settings);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer &telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();


private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;

  sick::datastructure::CommSettings m_settings;

  BYTE *prepareTelegramAndGetDataPtr(sick::datastructure::PacketBuffer::VectorBuffer &telegram) const;
  bool writeDataToDataPtr(BYTE *&data_ptr) const;
  bool writeChannelToDataPtr(BYTE *&data_ptr) const;
  bool writeEnabledToDataPtr(BYTE *&data_ptr) const;
  bool writeEInterfaceTypeToDataPtr(BYTE *&data_ptr) const;
  bool writeIPAdresstoDataPtr(BYTE *&data_ptr) const;
  bool writePortToDataPtr(BYTE *&data_ptr) const;
  bool writeFrequencyToDataPtr(BYTE *&data_ptr) const;
  bool writeStartAngleToDataPtr(BYTE *&data_ptr) const;
  bool writeEndAngleToDataPtr(BYTE *&data_ptr) const;
  bool writeFeaturesToDataPtr(BYTE *&data_ptr) const;
};

}
}
