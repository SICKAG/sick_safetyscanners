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

Command::Command(Cola2Session& session, const uint16_t& command_type, const uint16_t& command_mode)
  : m_session(session)
  , m_command_mode(command_mode)
  , m_command_type(command_type)
{
  m_session_id     = m_session.getSessionID();
  m_request_id     = m_session.getNextRequestID();
  m_tcp_parser_ptr = std::make_shared<sick::data_processing::ParseTCPPacket>();
  m_writer_ptr     = std::make_shared<sick::data_processing::ReadWriteHelper>();
}

void Command::lockExecutionMutex()
{
  m_execution_mutex.lock();
}

void Command::constructTelegram(datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  addTelegramData(telegram);
  addTelegramHeader(telegram);
}

void Command::processReplyBase(const datastructure::PacketBuffer::VectorBuffer& packet)
{
  m_tcp_parser_ptr->parseTCPSequence(packet, *this);
  m_was_successful = processReply();
  m_execution_mutex.unlock();
}

void Command::waitForCompletion()
{
  boost::mutex::scoped_lock lock(m_execution_mutex);
}

bool Command::wasSuccessful() const
{
  return m_was_successful;
}

uint8_t Command::getCommandType() const
{
  return m_command_type;
}

void Command::setCommandType(const uint8_t& command_type)
{
  m_command_type = command_type;
}

uint8_t Command::getCommandMode() const
{
  return m_command_mode;
}

void Command::setCommandMode(const uint8_t& command_mode)
{
  m_command_mode = command_mode;
}

uint32_t Command::getSessionID() const
{
  return m_session_id;
}

void Command::setSessionID(const uint32_t& session_id)
{
  m_session_id = session_id;
}

uint16_t Command::getRequestID() const
{
  return m_request_id;
}

void Command::setRequestID(const uint16_t& request_id)
{
  m_request_id = request_id;
}

void Command::addTelegramHeader(datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  datastructure::PacketBuffer::VectorBuffer header = prepareHeader();
  uint8_t* data_ptr                                = header.data();
  writeDataToDataPtr(data_ptr, telegram);
  telegram.insert(telegram.begin(), header.begin(), header.end());
}

sick::datastructure::PacketBuffer::VectorBuffer Command::prepareHeader() const
{
  datastructure::PacketBuffer::VectorBuffer header;
  header.resize(18);
  return header;
}

std::vector<uint8_t> Command::getDataVector() const
{
  return m_data_vector;
}

void Command::setDataVector(const std::vector<uint8_t>& data)
{
  m_data_vector = data;
}

void Command::writeDataToDataPtr(uint8_t*& data_ptr,
                                 datastructure::PacketBuffer::VectorBuffer& telegram) const
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

void Command::writeCola2StxToDataPtr(uint8_t*& data_ptr) const
{
  uint32_t cola2_stx = 0x02020202;
  m_writer_ptr->writeuint32_tBigEndian(data_ptr, cola2_stx, 0);
}

void Command::writeLengthToDataPtr(uint8_t*& data_ptr,
                                   datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  uint32_t length = 10 + telegram.size();
  m_writer_ptr->writeuint32_tBigEndian(data_ptr, length, 4);
}

void Command::writeCola2HubCntrToDataPtr(uint8_t*& data_ptr) const
{
  uint8_t cola2_hub_cntr = 0x00;
  m_writer_ptr->writeuint8_tBigEndian(data_ptr, cola2_hub_cntr, 8);
}

void Command::writeCola2NoCToDataPtr(uint8_t*& data_ptr) const
{
  uint8_t cola2_noc = 0x00;
  m_writer_ptr->writeuint8_tBigEndian(data_ptr, cola2_noc, 9);
}

void Command::writeSessionIdToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint32_tBigEndian(data_ptr, getSessionID(), 10);
}

void Command::writeRequestIdToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint16_tBigEndian(data_ptr, getRequestID(), 14);
}

void Command::writeCommandTypeToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint8_tBigEndian(data_ptr, getCommandType(), 16);
}

void Command::writeCommandModeToDataPtr(uint8_t*& data_ptr) const
{
  m_writer_ptr->writeuint8_tBigEndian(data_ptr, getCommandMode(), 17);
}

}  // namespace cola2
}  // namespace sick
