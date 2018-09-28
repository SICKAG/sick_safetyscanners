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
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();
}

int ParseTCPPacket::getExpectedPacketLength(datastructure::PacketBuffer buffer)
{
  return readLength(buffer) + 8; // for STX and Length which is not included in length datafield
}

UINT16 ParseTCPPacket::getRequestID(datastructure::PacketBuffer buffer)
{
  return readRequestID(buffer);
}


bool ParseTCPPacket::parseTCPSequence(datastructure::PacketBuffer buffer,
                                      sick::cola2::Command& command)
{
  setCommandValuesFromPacket(buffer, command);

  std::vector<BYTE> byteVector;
  readData(buffer, byteVector);
  command.setDataVector(byteVector);

  return true;
}

bool ParseTCPPacket::setCommandValuesFromPacket(sick::datastructure::PacketBuffer& buffer,
                                                sick::cola2::Command& command)
{
  command.setSessionID(readSessionID(buffer));
  command.setRequestID(readRequestID(buffer));
  command.setCommandType(readCommandType(buffer));
  command.setCommandMode(readCommandMode(buffer));
}

UINT32 ParseTCPPacket::readSTx(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT32BigEndian(data_ptr, 0);
}

UINT32 ParseTCPPacket::readLength(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT32BigEndian(data_ptr, 4);
}

UINT8 ParseTCPPacket::readHubCntr(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT8BigEndian(data_ptr, 8);
}
UINT8 ParseTCPPacket::readNoC(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT8BigEndian(data_ptr, 9);
}
UINT32 ParseTCPPacket::readSessionID(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT32BigEndian(data_ptr, 10);
}

UINT16 ParseTCPPacket::readRequestID(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT16BigEndian(data_ptr, 14);
}

UINT8 ParseTCPPacket::readCommandType(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT8BigEndian(data_ptr, 16);
}
UINT8 ParseTCPPacket::readCommandMode(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT8BigEndian(data_ptr, 17);
}
UINT16 ParseTCPPacket::readErrorCode(datastructure::PacketBuffer& buffer)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  return m_reader_ptr->readUINT16BigEndian(data_ptr, 18);
}

void ParseTCPPacket::readData(datastructure::PacketBuffer& buffer, std::vector<BYTE>& byteVector)
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
