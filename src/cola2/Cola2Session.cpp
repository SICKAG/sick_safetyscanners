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
* \file Cola2Session.cpp
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-09-24
*/
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>

namespace sick {
namespace cola2 {

Cola2Session::Cola2Session(boost::shared_ptr<sick::communication::AsyncTCPClient> async_tcp_client)
  : m_async_tcp_client_ptr(async_tcp_client)
  , m_session_id(0)
  , m_last_request_id(0)
{

  m_async_tcp_client_ptr->setPacketHandler(boost::bind(&Cola2Session::processPacket, this, _1));
  m_packet_merger_ptr = boost::make_shared<sick::data_processing::TCPPaketMerger>();
  m_tcp_parser_ptr = boost::make_shared<sick::data_processing::ParseTCPPacket>();

}

bool Cola2Session::open()
{
  CommandPtr command_ptr = boost::make_shared<CreateSession>(boost::ref(*this));
  return executeCommand(command_ptr);
}

bool Cola2Session::close()
{
  CommandPtr command_ptr = boost::make_shared<CloseSession>(boost::ref(*this));
  return executeCommand(command_ptr);
}

bool Cola2Session::executeCommand(CommandPtr command)
{
  //TODO sanitize
  addCommand(command->getRequestID(), command);
  sendTelegramAndListenForAnswer(command);
  return true;
}

bool Cola2Session::sendTelegramAndListenForAnswer(CommandPtr command)
{
  command->lockExecutionMutex(); //lock
  sick::datastructure::PacketBuffer::VectorBuffer telegram;
  command->constructTelegram(telegram);
  m_async_tcp_client_ptr->doSendAndReceive(telegram);
  command->waitForCompletion(); //scooped locked to wait, unlocked on data processing
}



UINT32 Cola2Session::getSessionID() const
{
  return m_session_id;
}

void Cola2Session::setSessionID(const UINT32 &session_id)
{
  m_session_id = session_id;
}

void Cola2Session::processPacket(const datastructure::PacketBuffer &packet)
{
  addPacketToMerger(packet);
  if (!checkIfPacketIsCompleteAndOtherwiseListenForMorePackets()) return;
  sick::datastructure::PacketBuffer deployedPacket = m_packet_merger_ptr->getDeployedPacketBuffer();
  startProcessingAndRemovePendingCommandAfterwards(deployedPacket);

}

bool Cola2Session::addPacketToMerger(const sick::datastructure::PacketBuffer &packet)
{
  if (m_packet_merger_ptr->isEmpty() || m_packet_merger_ptr->isComplete())
  {
    m_packet_merger_ptr->setTargetSize(m_tcp_parser_ptr->getExpectedPacketLength(packet));
  }
  m_packet_merger_ptr->addTCPPacket(packet);
  return true;
}

bool Cola2Session::checkIfPacketIsCompleteAndOtherwiseListenForMorePackets()
{
  if (!m_packet_merger_ptr->isComplete())
  {
    m_async_tcp_client_ptr->initiateReceive();
    return false;
  }
  return true;
}


bool Cola2Session::startProcessingAndRemovePendingCommandAfterwards(sick::datastructure::PacketBuffer &packet)
{
  UINT16 requestID = m_tcp_parser_ptr->getRequestID(packet);
  CommandPtr pendingCommand;
  if (findCommand(requestID, pendingCommand))
  {
    pendingCommand->processReplyBase(packet.getBuffer());
    removeCommand(requestID);
  }
}

bool Cola2Session::addCommand(UINT16 request_id, CommandPtr command)
{
  if(m_pending_commands_map.find(request_id) != m_pending_commands_map.end())
  {
    return false;
  }
  m_pending_commands_map[request_id] = command;
  return true;

}

bool Cola2Session::findCommand(UINT16 request_id, CommandPtr &command)
{
  if (m_pending_commands_map.find(request_id) == m_pending_commands_map.end())
  {
    return false;
  }
  command = m_pending_commands_map[request_id];
  return true;
}

bool Cola2Session::removeCommand(UINT16 request_id)
{
  auto it = m_pending_commands_map.find(request_id);
  if(it == m_pending_commands_map.end())
  {
    return false;
  }
  m_pending_commands_map.erase(it);
  return true;
}

UINT16 Cola2Session::getNextRequestID()
{
  if (m_last_request_id == std::numeric_limits<UINT16>::max())
  {
    m_last_request_id = 0;
  }
  return ++m_last_request_id;
}

}
}
