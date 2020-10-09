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
 * \file ParseMeasurementPersistentConfigData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-01-07
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseMeasurementPersistentConfigData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseMeasurementPersistentConfigData::ParseMeasurementPersistentConfigData() {}


bool ParseMeasurementPersistentConfigData::parseTCPSequence(
  const datastructure::PacketBuffer& buffer, datastructure::ConfigData& config_data) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  config_data.setVersionCVersion(readVersionIndicator(data_ptr));
  config_data.setVersionMajorVersionNumber(readMajorNumber(data_ptr));
  config_data.setVersionMinorVersionNumber(readMinorNumber(data_ptr));
  config_data.setVersionReleaseNumber(readReleaseNumber(data_ptr));
  config_data.setEnabled(readEnabled(data_ptr));
  config_data.setEInterfaceType(readInterfaceType(data_ptr));
  config_data.setHostIp(readHostIp(data_ptr));
  config_data.setHostUdpPort(readHostPort(data_ptr));
  config_data.setPublishingFrequency(readPublishingFreq(data_ptr));
  config_data.setStartAngle(readStartAngle(data_ptr));
  config_data.setEndAngle(readEndAngle(data_ptr));
  config_data.setFeatures(readFeatures(data_ptr)); // TODO
  return true;
}

std::string ParseMeasurementPersistentConfigData::readVersionIndicator(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::string result;
  result.push_back(read_write_helper::readUint8(data_ptr + 0));
  return result;
}

uint8_t ParseMeasurementPersistentConfigData::readMajorNumber(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 1);
}

uint8_t ParseMeasurementPersistentConfigData::readMinorNumber(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 2);
}

uint8_t ParseMeasurementPersistentConfigData::readReleaseNumber(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 3);
}

bool ParseMeasurementPersistentConfigData::readEnabled(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 4);
}

uint8_t ParseMeasurementPersistentConfigData::readInterfaceType(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 5);
}

boost::asio::ip::address_v4 ParseMeasurementPersistentConfigData::readHostIp(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint32_t word = read_write_helper::readUint32LittleEndian(data_ptr + 8);
  boost::asio::ip::address_v4 addr(word);
  return addr;
}

uint16_t ParseMeasurementPersistentConfigData::readHostPort(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 12);
}

uint16_t ParseMeasurementPersistentConfigData::readPublishingFreq(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 14);
}

uint32_t ParseMeasurementPersistentConfigData::readStartAngle(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 16);
}

uint32_t ParseMeasurementPersistentConfigData::readEndAngle(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 20);
}

uint16_t ParseMeasurementPersistentConfigData::readFeatures(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  // TODO parse Features
  return read_write_helper::readUint16LittleEndian(data_ptr + 24);
}


} // namespace data_processing
} // namespace sick
