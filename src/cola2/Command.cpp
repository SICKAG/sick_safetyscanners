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

#include <sick_safetyscanners/cola2/Command.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>


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
}

void Command::lockExecutionMutex()
{
  m_execution_mutex.lock();
}

std::vector<uint8_t> Command::constructTelegram(const std::vector<uint8_t>& telegram) const
{
  auto v = addTelegramData(telegram);
  return addTelegramHeader(v);
}

void Command::processReplyBase(const std::vector<uint8_t>& packet)
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

std::vector<uint8_t> Command::expandTelegram(const std::vector<uint8_t>& telegram,
                                             size_t additional_bytes) const
{
  // Allocate memory to the desired final size
  std::vector<uint8_t> output(telegram.size() + additional_bytes);
  // Copy the original telegram over second, this prevents potential reallocating if .resize() was
  // used
  std::copy(telegram.begin(), telegram.end(), output.begin());
  return output;
}

std::vector<uint8_t> Command::addTelegramHeader(const std::vector<uint8_t>& telegram) const
{
  std::vector<uint8_t> header             = prepareHeader();
  std::vector<uint8_t>::iterator data_ptr = header.begin();
  writeDataToDataPtr(data_ptr, telegram);
  // Add telegram to end of new header, this may resize header
  header.insert(header.end(), telegram.begin(), telegram.end());
  return header;
}

std::vector<uint8_t> Command::prepareHeader() const
{
  return std::vector<uint8_t>(18);
}

std::vector<uint8_t> Command::getDataVector() const
{
  return m_data_vector;
}

void Command::setDataVector(const std::vector<uint8_t>& data)
{
  m_data_vector = data;
}

void Command::writeDataToDataPtr(std::vector<uint8_t>::iterator data_ptr,
                                 const std::vector<uint8_t>& telegram) const
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

void Command::writeCola2StxToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  uint32_t cola2_stx = 0x02020202;
  read_write_helper::writeUint32BigEndian(data_ptr + 0, cola2_stx);
}

void Command::writeLengthToDataPtr(std::vector<uint8_t>::iterator data_ptr,
                                   const std::vector<uint8_t>& telegram) const
{
  uint32_t length = 10 + telegram.size();
  read_write_helper::writeUint32BigEndian(data_ptr + 4, length);
}

void Command::writeCola2HubCntrToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  uint8_t cola2_hub_cntr = 0x00;
  read_write_helper::writeUint8BigEndian(data_ptr + 8, cola2_hub_cntr);
}

void Command::writeCola2NoCToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  uint8_t cola2_noc = 0x00;
  read_write_helper::writeUint8BigEndian(data_ptr + 9, cola2_noc);
}

void Command::writeSessionIdToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint32BigEndian(data_ptr + 10, getSessionID());
}

void Command::writeRequestIdToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint16BigEndian(data_ptr + 14, getRequestID());
}

void Command::writeCommandTypeToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint8BigEndian(data_ptr + 16, getCommandType());
}

void Command::writeCommandModeToDataPtr(std::vector<uint8_t>::iterator data_ptr) const
{
  read_write_helper::writeUint8BigEndian(data_ptr + 17, getCommandMode());
}

} // namespace cola2
} // namespace sick
