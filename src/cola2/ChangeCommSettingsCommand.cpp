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
 * \file ChangeCommSettingsCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#include <sick_microscan3_ros_driver/cola2/ChangeCommSettingsCommand.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

ChangeCommSettingsCommand::ChangeCommSettingsCommand(
  Cola2Session& session, const sick::datastructure::CommSettings& settings)
  : MethodCommand(session, 0x00b0)
  , m_settings(settings)
{
  m_writer_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}

void ChangeCommSettingsCommand::addTelegramData(
  sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  base_class::addTelegramData(telegram);

  uint8_t* data_ptr = prepareTelegramAndGetDataPtr(telegram);

  writeDataToDataPtr(data_ptr);
}

uint8_t* ChangeCommSettingsCommand::prepareTelegramAndGetDataPtr(
  sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  uint16_t prevSize = telegram.size();
  telegram.resize(prevSize + 28);
  return telegram.data() + prevSize;
}

void ChangeCommSettingsCommand::writeDataToDataPtr(uint8_t*& data_ptr) const
{
  writeChannelToDataPtr(data_ptr);
  writeEnabledToDataPtr(data_ptr);
  writeEInterfaceTypeToDataPtr(data_ptr);
  writeIPAdresstoDataPtr(data_ptr);
  writePortToDataPtr(data_ptr);
  writeFrequencyToDataPtr(data_ptr);
  writeStartAngleToDataPtr(data_ptr);
  writeEndAngleToDataPtr(data_ptr);
  writeFeaturesToDataPtr(data_ptr);
}

bool ChangeCommSettingsCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool ChangeCommSettingsCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }
  return true;
}

void ChangeCommSettingsCommand::writeChannelToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint8_tLittleEndian(data_ptr, m_settings.getChannel(), 0);
}

void ChangeCommSettingsCommand::writeEnabledToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint8_tLittleEndian(data_ptr, m_settings.getEnabled(), 4);
}

void ChangeCommSettingsCommand::writeEInterfaceTypeToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint8_tLittleEndian(data_ptr, m_settings.getEInterfaceType(), 5);
}

void ChangeCommSettingsCommand::writeIPAdresstoDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint32_tLittleEndian(data_ptr, m_settings.getHostIp().to_ulong(), 8);
}

void ChangeCommSettingsCommand::writePortToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint16_tLittleEndian(data_ptr, m_settings.getHostUdpPort(), 12);
}

void ChangeCommSettingsCommand::writeFrequencyToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint16_tLittleEndian(data_ptr, m_settings.getPublishingFequency(), 14);
}

void ChangeCommSettingsCommand::writeStartAngleToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint32_tLittleEndian(data_ptr, m_settings.getStartAngle(), 16);
}

void ChangeCommSettingsCommand::writeEndAngleToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint32_tLittleEndian(data_ptr, m_settings.getEndAngle(), 20);
}

void ChangeCommSettingsCommand::writeFeaturesToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint16_tLittleEndian(data_ptr, m_settings.getFeatures(), 24);
}


} // namespace cola2
} // namespace sick
