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
 * \file AsyncTCPClient.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/communication/AsyncTCPClient.h>

namespace sick {
namespace communication {
AsyncTCPClient::AsyncTCPClient(PacketHandler packet_handler,
                               boost::asio::io_service& io_service,
                               boost::asio::ip::address_v4 server_ip,
                               unsigned short server_port)
  : m_packet_handler(packet_handler)
  , m_io_work_ptr()
  , m_io_service(io_service)

{
  // Keep io_service busy
  m_io_work_ptr = std::make_shared<boost::asio::io_service::work>(boost::ref(m_io_service));
  try
  {
    m_socket_ptr = std::make_shared<boost::asio::ip::tcp::socket>(boost::ref(m_io_service));
  }
  catch (std::exception& e)
  {
    std::cout << "Exception while creating socket: " << e.what() << std::endl;
  }
  m_remote_endpoint = boost::asio::ip::tcp::endpoint(server_ip, server_port);
  std::cout << "TCP client setup" << std::endl;
}

AsyncTCPClient::~AsyncTCPClient()
{
  m_socket_ptr->close();
}

void AsyncTCPClient::do_connect()
{
  boost::mutex::scoped_lock lock(m_connect_mutex);
  m_socket_ptr->async_connect(m_remote_endpoint, [this](boost::system::error_code ec) {
    std::cout << "TCP error code: " << ec.value() << std::endl;
    m_connect_condition.notify_all();
  });

  m_connect_condition.wait(lock);
}


void AsyncTCPClient::doSendAndReceive(
  const sick::datastructure::PacketBuffer::VectorBuffer& sendBuffer)
{
  if (!m_socket_ptr)
  {
    return;
  }
  boost::asio::async_write(*m_socket_ptr,
                           boost::asio::buffer(sendBuffer),
                           [this](boost::system::error_code ec, std::size_t bytes_send) {
                             this->handleSendAndReceive(ec, bytes_send);
                           });
}

void AsyncTCPClient::initiateReceive()
{
  if (!m_socket_ptr)
  {
    return;
  }
  m_socket_ptr->async_read_some(boost::asio::buffer(m_recv_buffer),
                                [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                                  this->handle_receive(ec, bytes_recvd);
                                });
}

void AsyncTCPClient::setPacketHandler(const PacketHandler& packet_handler)
{
  m_packet_handler = packet_handler;
}

void AsyncTCPClient::handleSendAndReceive(const boost::system::error_code& error,
                                          std::size_t bytes_transferred)
{
  // Check for errors
  if (!error || error == boost::asio::error::message_size)
  {
    initiateReceive();
  }
  else
  {
    std::cout << "Error in tcp handle send and receive: " << error.value() << std::endl;
  }
}


void AsyncTCPClient::start_receive() {}

void AsyncTCPClient::handle_receive(const boost::system::error_code& error,
                                    std::size_t bytes_transferred)
{
  if (!error)
  {
    sick::datastructure::PacketBuffer packet_buffer(m_recv_buffer, bytes_transferred);
    m_packet_handler(packet_buffer);
  }
  else
  {
    std::cout << "Error in tcp handle receive: " << error.value() << std::endl;
  }
}


void AsyncTCPClient::run_service() {}

} // namespace communication
} // namespace sick
