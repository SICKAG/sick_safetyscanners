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
 * \file AsyncTCPCLient.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COMMUNICATION_ASYNCTCPCLIENT_H
#define SICK_SAFETYSCANNERS_COMMUNICATION_ASYNCTCPCLIENT_H

#include <ros/ros.h>

#include <functional>
#include <iostream>
#include <thread>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/cstdint.hpp>
#include <boost/function.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include <sick_safetyscanners/datastructure/PacketBuffer.h>

namespace sick {
namespace communication {

/*!
 * \brief A asynchronous tcp client.
 *
 * Responsible to handle the connection to an IP-address and a port. Transfers the data to the
 * specified target and receives the answer. The answer will be passed to a packet handler.
 */
class AsyncTCPClient
{
public:
  /*!
   * \brief Typedef to a function referencing a packet handler. This can be passed to the class and
   * therefore be called on processing the data.
   */
  typedef boost::function<void(const sick::datastructure::PacketBuffer&)> PacketHandler;

  /*!
   * \brief Constructor of the asynchronous tcp client.
   *
   * \param packet_handler Function which handles the packets on processing.
   * \param io_service The boost io_service instance.
   * \param server_ip The IP address of the server to connect to.
   * \param server_port The port on the server to connect to.
   */
  AsyncTCPClient(const PacketHandler& packet_handler,
                 boost::asio::io_service& io_service,
                 const boost::asio::ip::address_v4& server_ip,
                 const uint16_t& server_port);

  /*!
   * \brief The destructor of the asynchronous tcp client.
   */
  virtual ~AsyncTCPClient();

  /*!
   * \brief Establishes a connection from the host to the sensor.
   */
  void doConnect();

  /*!
   * \brief Disconnects the host from the sensor
   */
  void doDisconnect();

  /*!
   * \brief Start a cycle of sensing a command and waiting got the return.
   *
   * \param sendBuffer The telegram which will be sent to the server.
   */
  void doSendAndReceive(const std::vector<uint8_t>& sendBuffer);

  /*!
   * \brief Initiates the listening for a message from the server.
   */
  void initiateReceive();

  /*!
   * \brief Sets the packet handler function.
   *
   * \param packet_handler The new packet handler function.
   */
  void setPacketHandler(const PacketHandler& packet_handler);

private:
  datastructure::PacketBuffer::ArrayBuffer m_recv_buffer;

  PacketHandler m_packet_handler;

  std::shared_ptr<boost::asio::io_service::work> m_io_work_ptr;
  boost::asio::io_service& m_io_service;
  std::shared_ptr<boost::asio::ip::tcp::socket> m_socket_ptr;
  boost::asio::ip::tcp::endpoint m_remote_endpoint;
  std::thread m_service_thread;

  boost::condition m_connect_condition;
  boost::mutex m_connect_mutex;
  boost::mutex m_socket_mutex;

  void startReceive();
  void handleReceive(const boost::system::error_code& error, const std::size_t& bytes_transferred);


  AsyncTCPClient(AsyncTCPClient&); // block default copy constructor

  void handleSendAndReceive(const boost::system::error_code& error,
                            const std::size_t& bytes_transferred);
};
} // namespace communication
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COMMUNICATION_ASYNCTCPCLIENT_H
