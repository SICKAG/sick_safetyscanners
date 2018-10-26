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

#include <sick_microscan3_ros_driver/data_processing/ParseTCPPacket.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseTCPPacket::ParseTCPPacket()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}

uint32_t ParseTCPPacket::getExpectedPacketLength(const datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return readLength(data_ptr) + 8; // for STX and Length which is not included in length datafield
}

uint16_t ParseTCPPacket::getRequestID(const datastructure::PacketBuffer& buffer) const
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return readRequestID(data_ptr);
}


bool ParseTCPPacket::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                      sick::cola2::Command& command) const
{
  setCommandValuesFromPacket(buffer, command);

  std::vector<uint8_t> byteVector;
  readData(buffer, byteVector);
  command.setDataVector(byteVector);

  return true;
}

void ParseTCPPacket::setCommandValuesFromPacket(const sick::datastructure::PacketBuffer& buffer,
                                                sick::cola2::Command& command) const
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  command.setSessionID(readSessionID(data_ptr));
  command.setRequestID(readRequestID(data_ptr));
  command.setCommandType(readCommandType(data_ptr));
  command.setCommandMode(readCommandMode(data_ptr));
}

uint32_t ParseTCPPacket::readSTx(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint32_tBigEndian(data_ptr, 0);
}

uint32_t ParseTCPPacket::readLength(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint32_tBigEndian(data_ptr, 4);
}

uint8_t ParseTCPPacket::readHubCntr(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 8);
}
uint8_t ParseTCPPacket::readNoC(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 9);
}
uint32_t ParseTCPPacket::readSessionID(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint32_tBigEndian(data_ptr, 10);
}

uint16_t ParseTCPPacket::readRequestID(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint16_tBigEndian(data_ptr, 14);
}

uint8_t ParseTCPPacket::readCommandType(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 16);
}
uint8_t ParseTCPPacket::readCommandMode(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 17);
}
uint16_t ParseTCPPacket::readErrorCode(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint16_tBigEndian(data_ptr, 18);
}

void ParseTCPPacket::readData(const datastructure::PacketBuffer& buffer,
                              std::vector<uint8_t>& byteVector) const
{
  if (buffer.getLength() < 20)
  {
    return;
  }
  else
  {
    byteVector.insert(byteVector.end(), buffer.getBuffer().begin() + 20, buffer.getBuffer().end());
  }
}

} // namespace data_processing
} // namespace sick
