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
 * \file ParseMeasurementCurrentConfigData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-01-07
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseMeasurementCurrentConfigData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseMeasurementCurrentConfigData::ParseMeasurementCurrentConfigData() {}


bool ParseMeasurementCurrentConfigData::parseTCPSequence(
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
  config_data.setDerivedMultiplicationFactor(readDerivedMultiplicationFactor(data_ptr));
  config_data.setDerivedNumberOfBeams(readDerivedNumBeams(data_ptr));
  config_data.setDerivedScanTime(readDerivedScanTime(data_ptr));
  config_data.setDerivedStartAngle(readDerivedStartAngle(data_ptr));
  config_data.setDerivedAngularBeamResolution(readDerivedAngularBeamResolution(data_ptr));
  config_data.setDerivedInterbeamPeriod(readDerivedInterbeamPeriod(data_ptr));
  return true;
}

std::string ParseMeasurementCurrentConfigData::readVersionIndicator(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::string result;
  result.push_back(read_write_helper::readUint8(data_ptr + 0));
  return result;
}

uint8_t ParseMeasurementCurrentConfigData::readMajorNumber(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 1);
}

uint8_t ParseMeasurementCurrentConfigData::readMinorNumber(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 2);
}

uint8_t ParseMeasurementCurrentConfigData::readReleaseNumber(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 3);
}

bool ParseMeasurementCurrentConfigData::readEnabled(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 4);
}

uint8_t ParseMeasurementCurrentConfigData::readInterfaceType(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 5);
}

boost::asio::ip::address_v4
ParseMeasurementCurrentConfigData::readHostIp(std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint32_t word = read_write_helper::readUint32LittleEndian(data_ptr + 8);
  boost::asio::ip::address_v4 addr(word);
  return addr;
}

uint16_t
ParseMeasurementCurrentConfigData::readHostPort(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 12);
}

uint16_t ParseMeasurementCurrentConfigData::readPublishingFreq(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 14);
}

int32_t ParseMeasurementCurrentConfigData::readStartAngle(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readInt32LittleEndian(data_ptr + 16);
}

int32_t
ParseMeasurementCurrentConfigData::readEndAngle(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readInt32LittleEndian(data_ptr + 20);
}

uint16_t
ParseMeasurementCurrentConfigData::readFeatures(std::vector<uint8_t>::const_iterator data_ptr) const
{
  // TODO parse Features
  return read_write_helper::readUint16LittleEndian(data_ptr + 24);
}

uint16_t ParseMeasurementCurrentConfigData::readDerivedMultiplicationFactor(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 28);
}

uint16_t ParseMeasurementCurrentConfigData::readDerivedNumBeams(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 30);
}

uint16_t ParseMeasurementCurrentConfigData::readDerivedScanTime(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 32);
}

int32_t ParseMeasurementCurrentConfigData::readDerivedStartAngle(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readInt32LittleEndian(data_ptr + 36);
}

int32_t ParseMeasurementCurrentConfigData::readDerivedAngularBeamResolution(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readInt32LittleEndian(data_ptr + 40);
}

uint32_t ParseMeasurementCurrentConfigData::readDerivedInterbeamPeriod(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 44);
}


} // namespace data_processing
} // namespace sick
