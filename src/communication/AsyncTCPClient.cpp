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

#include <sick_safetyscanners/communication/AsyncTCPClient.h>

namespace sick {
namespace communication {
AsyncTCPClient::AsyncTCPClient(const PacketHandler& packet_handler,
                               boost::asio::io_service& io_service,
                               const boost::asio::ip::address_v4& server_ip,
                               const uint16_t& server_port)
  : m_packet_handler(packet_handler)
  , m_io_service(io_service)

{
  // Keep io_service busy
  m_io_work_ptr = std::make_shared<boost::asio::io_service::work>(m_io_service);
  try
  {
    m_socket_ptr = std::make_shared<boost::asio::ip::tcp::socket>(m_io_service);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception while creating socket: %s", e.what());
  }
  m_remote_endpoint = boost::asio::ip::tcp::endpoint(server_ip, server_port);
  ROS_INFO("TCP client is setup");
}

AsyncTCPClient::~AsyncTCPClient() {}

void AsyncTCPClient::doDisconnect()
{
  boost::mutex::scoped_lock lock(m_socket_mutex);
  boost::system::error_code ec;
  m_socket_ptr->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
  if (ec != boost::system::errc::success)
  {
    ROS_ERROR("Error shutting socket down: %i", ec.value());
  }
  else
  {
    ROS_INFO("TCP Connection successfully shutdown");
  }

  m_socket_ptr->close(ec);
  if (ec != boost::system::errc::success)
  {
    ROS_ERROR("Error closing Socket: %i", ec.value());
  }
  else
  {
    ROS_INFO("TCP Socket successfully closed.");
  }
}

void AsyncTCPClient::doConnect()
{
  boost::mutex::scoped_lock lock(m_socket_mutex);
  boost::mutex::scoped_lock lock_connect(m_connect_mutex);
  m_socket_ptr->async_connect(m_remote_endpoint, [this](boost::system::error_code ec) {
    if (ec != boost::system::errc::success)
    {
      ROS_ERROR("TCP error code: %i", ec.value());
    }
    else
    {
      ROS_INFO("TCP connection successfully established.");
    }
    m_connect_condition.notify_all();
  });

  m_connect_condition.wait(lock_connect);
}


void AsyncTCPClient::doSendAndReceive(const std::vector<uint8_t>& sendBuffer)
{
  boost::mutex::scoped_lock lock(m_socket_mutex);
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
  boost::mutex::scoped_lock lock(m_socket_mutex);
  if (!m_socket_ptr)
  {
    return;
  }
  m_socket_ptr->async_read_some(boost::asio::buffer(m_recv_buffer),
                                [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                                  this->handleReceive(ec, bytes_recvd);
                                });
}

void AsyncTCPClient::setPacketHandler(const PacketHandler& packet_handler)
{
  m_packet_handler = packet_handler;
}

void AsyncTCPClient::handleSendAndReceive(const boost::system::error_code& error,
                                          const std::size_t& bytes_transferred)
{
  // Check for errors
  if (!error || error == boost::asio::error::message_size)
  {
    initiateReceive();
  }
  else
  {
    ROS_ERROR("Error in tcp handle send and receive: %i", error.value());
  }
}


void AsyncTCPClient::startReceive() {}

void AsyncTCPClient::handleReceive(const boost::system::error_code& error,
                                   const std::size_t& bytes_transferred)
{
  if (!error)
  {
    sick::datastructure::PacketBuffer packet_buffer(m_recv_buffer, bytes_transferred);
    m_packet_handler(packet_buffer);
  }
  else
  {
    ROS_ERROR("Error in tcp handle receive: %i", error.value());
  }
}


} // namespace communication
} // namespace sick
