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
 * \file Command.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef COMMAND_H
#define COMMAND_H

#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ParseTCPPacket.h>
#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

#include <boost/thread/mutex.hpp>

namespace sick {
namespace cola2 {

class Cola2Session;

class Command
{
public:
  Command(sick::cola2::Cola2Session& session, uint16_t command_type, uint16_t command_mode);


  void lockExecutionMutex();

  void constructTelegram(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;

  void processReplyBase(const sick::datastructure::PacketBuffer::VectorBuffer& packet);

  void waitForCompletion();

  uint32_t getSessionID() const;
  void setSessionID(const uint32_t& session_id);


  bool wasSuccessful() const;


  uint8_t getCommandType() const;
  void setCommandType(const uint8_t& command_type);

  uint8_t getCommandMode() const;
  void setCommandMode(const uint8_t& command_mode);

  uint16_t getRequestID() const;
  void setRequestID(const uint16_t& request_id);

  std::vector<uint8_t> getDataVector() const;
  void setDataVector(const std::vector<uint8_t>& data);

protected:
  sick::cola2::Cola2Session& m_session;

private:
  boost::shared_ptr<sick::data_processing::ParseTCPPacket> m_tcp_parser_ptr;
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;


  virtual bool processReply() = 0;

  void addTelegramHeader(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  virtual void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const = 0;


  boost::mutex m_execution_mutex;

  bool m_was_successful;

  uint8_t m_command_mode;
  uint8_t m_command_type;

  uint32_t m_session_id;
  uint16_t m_request_id;

  std::vector<uint8_t> m_data_vector;
  sick::datastructure::PacketBuffer::VectorBuffer prepareHeader() const;
  void writeCola2StxToDataPtr(uint8_t*& data_ptr) const;
  void writeLengthToDataPtr(uint8_t*& data_ptr,
                            datastructure::PacketBuffer::VectorBuffer& telegram) const;
  void writeCola2HubCntrToDataPtr(uint8_t*& data_ptr) const;
  void writeCola2NoCToDataPtr(uint8_t*& data_ptr) const;
  void writeSessionIdToDataPtr(uint8_t*& data_ptr) const;
  void writeRequestIdToDataPtr(uint8_t*& data_ptr) const;
  void writeCommandTypeToDataPtr(uint8_t*& data_ptr) const;
  void writeCommandModeToDataPtr(uint8_t*& data_ptr) const;
  void writeDataToDataPtr(uint8_t*& data_ptr,
                          datastructure::PacketBuffer::VectorBuffer& telegram) const;
};


} // namespace cola2
} // namespace sick


#endif
