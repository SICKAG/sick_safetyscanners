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
 * \file Microscan3.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#include "sick_microscan3_ros_driver/Microscan3.h"

namespace sick {

Microscan3::Microscan3(PaketReceivedCallbackFunction newPaketReceivedCallbackFunction,
                       sick::datastructure::CommSettings settings)
  : m_newPaketReceivedCallbackFunction(newPaketReceivedCallbackFunction)
{
  std::cout << "starting MicroScan3" << std::endl;
  m_io_service_ptr       = std::make_shared<boost::asio::io_service>();
  m_async_udp_client_ptr = std::make_shared<sick::communication::AsyncUDPClient>(
    boost::bind(&Microscan3::processUDPPaket, this, _1),
    boost::ref(*m_io_service_ptr),
    settings.getHostUdpPort());
  m_paket_merger_ptr = std::make_shared<sick::data_processing::UDPPaketMerger>();
  std::cout << "started MicroScan3" << std::endl;
}

Microscan3::~Microscan3()
{
  m_udp_client_thread_ptr.reset();
}

bool Microscan3::run()
{
  m_udp_client_thread_ptr.reset(new boost::thread(boost::bind(&Microscan3::UDPClientThread, this)));

  m_async_udp_client_ptr->run_service();
  return true;
}

bool Microscan3::UDPClientThread()
{
  std::cout << "Enter io thread" << std::endl;
  m_io_work_ptr = std::make_shared<boost::asio::io_service::work>(boost::ref(*m_io_service_ptr));
  m_io_service_ptr->run();
  std::cout << "Exit io thread" << std::endl;
  return true;
}


void Microscan3::processTCPPaket(const sick::datastructure::PacketBuffer& buffer)
{
  // Not needed for current functionality, inplace for possible future developments
}

void Microscan3::changeSensorSettings(sick::datastructure::CommSettings settings)
{
  startTCPConnection(settings);

  changeCommSettingsinColaSession(settings);

  stopTCPConnection();
}

void Microscan3::startTCPConnection(sick::datastructure::CommSettings settings)
{
  std::shared_ptr<sick::communication::AsyncTCPClient> async_tcp_client =
    std::make_shared<sick::communication::AsyncTCPClient>(
      boost::bind(&Microscan3::processTCPPaket, this, _1),
      boost::ref(*m_io_service_ptr),
      settings.getSensorIp(),
      settings.getSensorTcpPort());
  async_tcp_client->do_connect();
  m_session_ptr = std::make_shared<sick::cola2::Cola2Session>(async_tcp_client);
}

void Microscan3::changeCommSettingsinColaSession(sick::datastructure::CommSettings settings)
{
  m_session_ptr->open();
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ChangeCommSettingsCommand>(boost::ref(*m_session_ptr),
                                                               settings);
  m_session_ptr->executeCommand(command_ptr);
  m_session_ptr->close();
}

void Microscan3::stopTCPConnection()
{
  m_session_ptr.reset();
}


void Microscan3::processUDPPaket(const sick::datastructure::PacketBuffer& buffer)
{
  if (m_paket_merger_ptr->addUDPPaket(buffer))
  {
    sick::datastructure::PacketBuffer deployedBuffer =
      m_paket_merger_ptr->getDeployedPacketBuffer();
    sick::datastructure::Data data;
    sick::data_processing::ParseData data_parser;
    data_parser.parseUDPSequence(deployedBuffer, data);

    m_newPaketReceivedCallbackFunction(data);
  }
}

} // namespace sick
