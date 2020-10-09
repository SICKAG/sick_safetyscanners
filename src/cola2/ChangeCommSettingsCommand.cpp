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


#include <sick_safetyscanners/cola2/ChangeCommSettingsCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

ChangeCommSettingsCommand::ChangeCommSettingsCommand(
  Cola2Session& session, const sick::datastructure::CommSettings& settings)
  : MethodCommand(session, 0x00b0)
  , m_settings(settings)
{
}

std::vector<uint8_t>
ChangeCommSettingsCommand::addTelegramData(const std::vector<uint8_t>& telegram) const
{
  auto base_output   = base_class::addTelegramData(telegram);
  size_t base_length = base_output.size();
  auto output        = expandTelegram(base_output, 28);

  // Add new values after telegram
  const auto new_data_offset_it = output.begin() + base_length + telegram.size();

  writeDataToDataPtr(new_data_offset_it);

  return output;
}

void ChangeCommSettingsCommand::writeDataToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  writeChannelToDataPtr(data_ptr);
  writeEnabledToDataPtr(data_ptr);
  writeEInterfaceTypeToDataPtr(data_ptr);
  writeIPAddresstoDataPtr(data_ptr);
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
  return (!base_class::processReply());
}

void ChangeCommSettingsCommand::writeChannelToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint8LittleEndian(data_ptr + 0, m_settings.getChannel());
}

void ChangeCommSettingsCommand::writeEnabledToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint8LittleEndian(data_ptr + 4,
                                            static_cast<uint8_t>(m_settings.getEnabled()));
}

void ChangeCommSettingsCommand::writeEInterfaceTypeToDataPtr(
  std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint8LittleEndian(data_ptr + 5, m_settings.getEInterfaceType());
}

void ChangeCommSettingsCommand::writeIPAddresstoDataPtr(
  std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint32LittleEndian(data_ptr + 8, m_settings.getHostIp().to_ulong());
}

void ChangeCommSettingsCommand::writePortToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint16LittleEndian(data_ptr + 12, m_settings.getHostUdpPort());
}

void ChangeCommSettingsCommand::writeFrequencyToDataPtr(
  std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint16LittleEndian(data_ptr + 14, m_settings.getPublishingFrequency());
}

void ChangeCommSettingsCommand::writeStartAngleToDataPtr(
  std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint32LittleEndian(data_ptr + 16, m_settings.getStartAngle());
}

void ChangeCommSettingsCommand::writeEndAngleToDataPtr(
  std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint32LittleEndian(data_ptr + 20, m_settings.getEndAngle());
}

void ChangeCommSettingsCommand::writeFeaturesToDataPtr(
  std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint16LittleEndian(data_ptr + 24, m_settings.getFeatures());
}


} // namespace cola2
} // namespace sick
