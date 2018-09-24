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
* \file Command.cpp
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-09-24
*/
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/cola2/Command.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>


namespace sick {
namespace cola2 {

Command::Command(Cola2Session &session, UINT16 command_type, UINT16 command_mode)
  : m_session(session)
  , m_command_mode(command_mode)
  , m_command_type(command_type)
{
  m_session_id = m_session.getSessionID();
  m_request_id = m_session.getNextRequestID();
  m_tcp_parser_ptr = boost::make_shared<sick::data_processing::ParseTCPPacket>();
  m_writer_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

void Command::lockExecutionMutex()
{
  m_execution_mutex.lock();
}

void Command::constructTelegram(datastructure::PacketBuffer::VectorBuffer &telegram) const
{
  addTelegramData(telegram);
  addTelegramHeader(telegram);
}

void Command::processReplyBase(const datastructure::PacketBuffer::VectorBuffer &packet)
{
  m_tcp_parser_ptr->parseTCPSequence(packet, *this);
  m_was_successful = processReply();
  m_execution_mutex.unlock();
}

void Command::waitForCompletion()
{
  boost::mutex::scoped_lock lock(m_execution_mutex);
}

bool Command::wasSuccessful() const {return m_was_successful;}

UINT8 Command::getCommandType() const
{
  return m_command_type;
}

void Command::setCommandType(const UINT8 &command_type)
{
  m_command_type = command_type;
}

UINT8 Command::getCommandMode() const
{
  return m_command_mode;
}

void Command::setCommandMode(const UINT8 &command_mode)
{
  m_command_mode = command_mode;
}

UINT32 Command::getSessionID() const
{
  return m_session_id;
}

void Command::setSessionID(const UINT32 &session_id)
{
  m_session_id = session_id;
}

UINT16 Command::getRequestID() const
{
  return m_request_id;
}

void Command::setRequestID(const UINT16 &request_id)
{
  m_request_id = request_id;
}

void Command::addTelegramHeader(datastructure::PacketBuffer::VectorBuffer &telegram) const
{
  datastructure::PacketBuffer::VectorBuffer header = prepareHeader();
  BYTE* data_ptr = header.data();
  writeDataToDataPtr(data_ptr, telegram);
  telegram.insert(telegram.begin(), header.begin(), header.end());
}

sick::datastructure::PacketBuffer::VectorBuffer Command::prepareHeader()  const
{
  datastructure::PacketBuffer::VectorBuffer header;
  header.resize(18);
  return header;
}

std::vector<BYTE> Command::getDataVector() const
{
  return m_data_vector;
}

void Command::setDataVector(const std::vector<BYTE> &data)
{
  m_data_vector = data;
}

bool Command::writeDataToDataPtr(BYTE*& data_ptr, datastructure::PacketBuffer::VectorBuffer &telegram) const
{
  writeCola2StxToDataPtr(data_ptr);
  writeLengthToDataPtr(data_ptr, telegram);
  writeCola2HubCntrToDataPtr(data_ptr);
  writeCola2NoCToDataPtr(data_ptr);
  writeSessionIdToDataPtr(data_ptr);
  writeRequestIdToDataPtr(data_ptr);
  writeCommandTypeToDataPtr(data_ptr);
  writeCommandModeToDataPtr(data_ptr);
}

bool Command::writeCola2StxToDataPtr(BYTE*& data_ptr) const
{
  UINT32 cola2_stx = 0x02020202;
  m_writer_ptr->writeUINT32BigEndian(data_ptr, cola2_stx, 0);
}

bool Command::writeLengthToDataPtr(BYTE*& data_ptr, datastructure::PacketBuffer::VectorBuffer &telegram) const
{
  UINT32 length = 10 + telegram.size();
  m_writer_ptr->writeUINT32BigEndian(data_ptr, length, 4);
}

bool Command::writeCola2HubCntrToDataPtr(BYTE*& data_ptr) const
{
  UINT8 cola2_hub_cntr = 0x00;
  m_writer_ptr->writeUINT8BigEndian(data_ptr, cola2_hub_cntr, 8);
}

bool Command::writeCola2NoCToDataPtr(BYTE*& data_ptr) const
{
  UINT8 cola2_noc = 0x00;
  m_writer_ptr->writeUINT8BigEndian(data_ptr, cola2_noc, 9);
}

bool Command::writeSessionIdToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT32BigEndian(data_ptr, getSessionID(),  10);
}

bool Command::writeRequestIdToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT16BigEndian(data_ptr, getRequestID(),14);
}

bool Command::writeCommandTypeToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT8BigEndian(data_ptr, getCommandType(),16);
}

bool Command::writeCommandModeToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT8BigEndian(data_ptr, getCommandMode(),17);
}

}
}

