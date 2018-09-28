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
 * \file AsyncUDPClient.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#pragma once

#include <functional>
#include <iostream>
#include <thread>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/cstdint.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>


namespace sick {
namespace communication {
class AsyncUDPClient
{
public:
  typedef boost::function<void(const sick::datastructure::PacketBuffer&)> PacketHandler;

  AsyncUDPClient(PacketHandler packet_handler,
                 boost::asio::io_service& io_service,
                 unsigned short local_port = 0);
  virtual ~AsyncUDPClient();

  void run_service();

private:
  datastructure::PacketBuffer::ArrayBuffer m_recv_buffer;

  PacketHandler m_packet_handler;

  boost::shared_ptr<boost::asio::io_service::work> m_io_work_ptr;
  boost::asio::io_service& m_io_service;
  boost::shared_ptr<boost::asio::ip::udp::socket> m_socket_ptr;
  boost::asio::ip::udp::endpoint m_remote_endpoint;
  std::thread m_service_thread;

  void start_receive();
  void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);


  AsyncUDPClient(AsyncUDPClient&); // block default copy constructor
};
} // namespace communication
} // namespace sick
