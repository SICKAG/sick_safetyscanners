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


#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
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
  Command(sick::cola2::Cola2Session& session, UINT16 command_type, UINT16 command_mode);


  void lockExecutionMutex();

  void constructTelegram(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;

  void processReplyBase(const sick::datastructure::PacketBuffer::VectorBuffer& packet);

  void waitForCompletion();

  UINT32 getSessionID() const;
  void setSessionID(const UINT32& session_id);


  bool wasSuccessful() const;


  UINT8 getCommandType() const;
  void setCommandType(const UINT8& command_type);

  UINT8 getCommandMode() const;
  void setCommandMode(const UINT8& command_mode);

  UINT16 getRequestID() const;
  void setRequestID(const UINT16& request_id);

  std::vector<BYTE> getDataVector() const;
  void setDataVector(const std::vector<BYTE>& data);

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

  UINT8 m_command_mode;
  UINT8 m_command_type;

  UINT32 m_session_id;
  UINT16 m_request_id;

  std::vector<BYTE> m_data_vector;
  sick::datastructure::PacketBuffer::VectorBuffer prepareHeader() const;
  void writeCola2StxToDataPtr(BYTE*& data_ptr) const;
  void writeLengthToDataPtr(BYTE*& data_ptr,
                            datastructure::PacketBuffer::VectorBuffer& telegram) const;
  void writeCola2HubCntrToDataPtr(BYTE*& data_ptr) const;
  void writeCola2NoCToDataPtr(BYTE*& data_ptr) const;
  void writeSessionIdToDataPtr(BYTE*& data_ptr) const;
  void writeRequestIdToDataPtr(BYTE*& data_ptr) const;
  void writeCommandTypeToDataPtr(BYTE*& data_ptr) const;
  void writeCommandModeToDataPtr(BYTE*& data_ptr) const;
  void writeDataToDataPtr(BYTE*& data_ptr,
                          datastructure::PacketBuffer::VectorBuffer& telegram) const;
};


} // namespace cola2
} // namespace sick


#endif
