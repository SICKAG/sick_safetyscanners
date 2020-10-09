// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2019, SICK AG, Waldkirch
*  Copyright (C) 2019, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file ParseStatusOverview.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-22
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseStatusOverview.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseStatusOverviewData::ParseStatusOverviewData() {}


bool ParseStatusOverviewData::parseTCPSequence(
  const datastructure::PacketBuffer& buffer,
  sick::datastructure::StatusOverview& status_overview) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  status_overview.setVersionCVersion(readVersionIndicator(data_ptr));
  status_overview.setVersionMajorVersionNumber(readMajorNumber(data_ptr));
  status_overview.setVersionMinorVersionNumber(readMinorNumber(data_ptr));
  status_overview.setVersionReleaseNumber(readReleaseNumber(data_ptr));
  status_overview.setDeviceState(readDeviceState(data_ptr));
  status_overview.setConfigState(readConfigState(data_ptr));
  status_overview.setApplicationState(readApplicationState(data_ptr));
  status_overview.setCurrentTimePowerOnCount(readPowerOnCount(data_ptr));
  status_overview.setCurrentTimeTime(readCurrentTime(data_ptr));
  status_overview.setCurrentTimeDate(readCurrentDate(data_ptr));
  status_overview.setErrorInfoCode(readErrorInfoCode(data_ptr));
  status_overview.setErrorInfoTime(readErrorInfoTime(data_ptr));
  status_overview.setErrorInfoDate(readErrorInfoDate(data_ptr));
  return true;
}

std::string
ParseStatusOverviewData::readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::string result;
  result.push_back(read_write_helper::readUint8(data_ptr + 0));
  return result;
}

uint8_t
ParseStatusOverviewData::readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 1);
}

uint8_t
ParseStatusOverviewData::readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 2);
}

uint8_t
ParseStatusOverviewData::readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 3);
}

uint8_t
ParseStatusOverviewData::readDeviceState(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 4);
}

uint8_t
ParseStatusOverviewData::readConfigState(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 5);
}

uint8_t
ParseStatusOverviewData::readApplicationState(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 6);
}

uint32_t
ParseStatusOverviewData::readPowerOnCount(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 12);
}

uint32_t
ParseStatusOverviewData::readCurrentTime(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 16);
}

uint16_t
ParseStatusOverviewData::readCurrentDate(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 20);
}

uint32_t
ParseStatusOverviewData::readErrorInfoCode(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 24);
}

uint32_t
ParseStatusOverviewData::readErrorInfoTime(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 52);
}

uint16_t
ParseStatusOverviewData::readErrorInfoDate(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 56);
}


} // namespace data_processing
} // namespace sick
