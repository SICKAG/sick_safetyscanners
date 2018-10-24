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

int ParseTCPPacket::getExpectedPacketLength(datastructure::PacketBuffer buffer)
{
  return readLength(buffer) + 8; // for STX and Length which is not included in length datafield
}

uint16_t ParseTCPPacket::getRequestID(datastructure::PacketBuffer buffer)
{
  return readRequestID(buffer);
}


bool ParseTCPPacket::parseTCPSequence(datastructure::PacketBuffer buffer,
                                      sick::cola2::Command& command)
{

  //TODO replace buffer with data ptr
  setCommandValuesFromPacket(buffer, command);

  std::vector<uint8_t> byteVector;
  readData(buffer, byteVector);
  command.setDataVector(byteVector);

  return true;
}

void ParseTCPPacket::setCommandValuesFromPacket(sick::datastructure::PacketBuffer& buffer,
                                                sick::cola2::Command& command)
{
  command.setSessionID(readSessionID(buffer));
  command.setRequestID(readRequestID(buffer));
  command.setCommandType(readCommandType(buffer));
  command.setCommandMode(readCommandMode(buffer));
}

uint32_t ParseTCPPacket::readSTx(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint32_tBigEndian(data_ptr, 0);
}

uint32_t ParseTCPPacket::readLength(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint32_tBigEndian(data_ptr, 4);
}

uint8_t ParseTCPPacket::readHubCntr(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 8);
}
uint8_t ParseTCPPacket::readNoC(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 9);
}
uint32_t ParseTCPPacket::readSessionID(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint32_tBigEndian(data_ptr, 10);
}

uint16_t ParseTCPPacket::readRequestID(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint16_tBigEndian(data_ptr, 14);
}

uint8_t ParseTCPPacket::readCommandType(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 16);
}
uint8_t ParseTCPPacket::readCommandMode(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint8_tBigEndian(data_ptr, 17);
}
uint16_t ParseTCPPacket::readErrorCode(datastructure::PacketBuffer& buffer)
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readuint16_tBigEndian(data_ptr, 18);
}

void ParseTCPPacket::readData(datastructure::PacketBuffer& buffer, std::vector<uint8_t>& byteVector)
{
  if (buffer.getLength() == 18)
  {
    return;
  }
  else
  {
    byteVector.insert(byteVector.end(), buffer.getBuffer().begin() + 18, buffer.getBuffer().end());
  }
}

} // namespace data_processing
} // namespace sick
