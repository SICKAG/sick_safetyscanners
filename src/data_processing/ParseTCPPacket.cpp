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
 * \file ParseTCPPacket.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseTCPPacket.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseTCPPacket::ParseTCPPacket() {}

uint32_t ParseTCPPacket::getExpectedPacketLength(const datastructure::PacketBuffer& buffer)
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  return readLength(data_ptr) + 8; // for STX and Length which is not included in length datafield
}

uint16_t ParseTCPPacket::getRequestID(const datastructure::PacketBuffer& buffer) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  return readRequestID(data_ptr);
}


bool ParseTCPPacket::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                      sick::cola2::Command& command) const
{
  setCommandValuesFromPacket(buffer, command);

  std::vector<uint8_t> byte_vector = readData(buffer);
  command.setDataVector(byte_vector);

  return true;
}

void ParseTCPPacket::setCommandValuesFromPacket(const sick::datastructure::PacketBuffer& buffer,
                                                sick::cola2::Command& command) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  command.setSessionID(readSessionID(data_ptr));
  command.setRequestID(readRequestID(data_ptr));
  command.setCommandType(readCommandType(data_ptr));
  command.setCommandMode(readCommandMode(data_ptr));
}

uint32_t ParseTCPPacket::readSTx(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32BigEndian(data_ptr + 0);
}

uint32_t ParseTCPPacket::readLength(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32BigEndian(data_ptr + 4);
}

uint8_t ParseTCPPacket::readHubCntr(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8BigEndian(data_ptr + 8);
}
uint8_t ParseTCPPacket::readNoC(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8BigEndian(data_ptr + 9);
}
uint32_t ParseTCPPacket::readSessionID(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32BigEndian(data_ptr + 10);
}

uint16_t ParseTCPPacket::readRequestID(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16BigEndian(data_ptr + 14);
}

uint8_t ParseTCPPacket::readCommandType(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8BigEndian(data_ptr + 16);
}
uint8_t ParseTCPPacket::readCommandMode(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8BigEndian(data_ptr + 17);
}
uint16_t ParseTCPPacket::readErrorCode(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint16BigEndian(data_ptr + 18);
}

std::vector<uint8_t> ParseTCPPacket::readData(const datastructure::PacketBuffer& buffer) const
{
  if (buffer.getLength() < 20)
  {
    return std::vector<uint8_t>();
  }
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  return std::vector<uint8_t>(vec_ptr->begin() + 20, vec_ptr->end());
}

} // namespace data_processing
} // namespace sick
