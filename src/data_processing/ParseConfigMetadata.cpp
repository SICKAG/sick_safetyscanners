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
 * \file ParseConfigMetadata.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-22
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseConfigMetadata.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseConfigMetadata::ParseConfigMetadata() {}


bool ParseConfigMetadata::parseTCPSequence(
  const datastructure::PacketBuffer& buffer,
  sick::datastructure::ConfigMetadata& config_metadata) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  config_metadata.setVersionCVersion(readVersionIndicator(data_ptr));
  config_metadata.setVersionMajorVersionNumber(readMajorNumber(data_ptr));
  config_metadata.setVersionMinorVersionNumber(readMinorNumber(data_ptr));
  config_metadata.setVersionReleaseNumber(readReleaseNumber(data_ptr));
  config_metadata.setModificationTimeDate(readModificationTimeDate(data_ptr));
  config_metadata.setModificationTimeTime(readModificationTimeTime(data_ptr));
  config_metadata.setTransferTimeDate(readTransferTimeDate(data_ptr));
  config_metadata.setTransferTimeTime(readTransferTimeTime(data_ptr));
  config_metadata.setAppChecksum(readAppChecksum(data_ptr));
  config_metadata.setOverallChecksum(readOverallChecksum(data_ptr));
  config_metadata.setIntegrityHash(readIntegrityHash(data_ptr));
  return true;
}

std::string
ParseConfigMetadata::readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::string result;
  result.push_back(read_write_helper::readUint8(data_ptr + 0));
  return result;
}

uint8_t ParseConfigMetadata::readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 1);
}

uint8_t ParseConfigMetadata::readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 2);
}

uint8_t ParseConfigMetadata::readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 3);
}

uint16_t
ParseConfigMetadata::readModificationTimeDate(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 4);
}

uint32_t
ParseConfigMetadata::readModificationTimeTime(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 8);
}

uint16_t
ParseConfigMetadata::readTransferTimeDate(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16LittleEndian(data_ptr + 12);
}

uint32_t
ParseConfigMetadata::readTransferTimeTime(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 16);
}

uint32_t ParseConfigMetadata::readAppChecksum(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 36);
}

uint32_t
ParseConfigMetadata::readOverallChecksum(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 52);
}

std::vector<uint32_t>
ParseConfigMetadata::readIntegrityHash(std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::vector<uint32_t> result;
  for (uint8_t i = 0; i < 4; i++)
  {
    result.push_back(read_write_helper::readUint32LittleEndian(data_ptr + 68 + (i * 4)));
  }
  return result;
}

} // namespace data_processing
} // namespace sick
