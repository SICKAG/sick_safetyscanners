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
 * \file Microscan3.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef MICROSCAN3_H
#define MICROSCAN3_H

#include <ros/ros.h>

#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <iostream>

#include <sick_microscan3_ros_driver/communication/AsyncTCPClient.h>
#include <sick_microscan3_ros_driver/communication/AsyncUDPClient.h>
#include <sick_microscan3_ros_driver/data_processing/ParseData.h>
#include <sick_microscan3_ros_driver/data_processing/UDPPacketMerger.h>
#include <sick_microscan3_ros_driver/datastructure/CommSettings.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/cola2/ChangeCommSettingsCommand.h>
#include <sick_microscan3_ros_driver/cola2/TypeCodeVariableCommand.h>
#include <sick_microscan3_ros_driver/cola2/FieldHeaderVariableCommand.h>
#include <sick_microscan3_ros_driver/cola2/FieldGeometryVariableCommand.h>
#include <sick_microscan3_ros_driver/cola2/MonitoringCaseTableHeaderVariableCommand.h>
#include <sick_microscan3_ros_driver/cola2/DeviceNameVariableCommand.h>
#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>

namespace sick {

/*!
 * @brief Class managing the algorithmic part of the package.
 */
class Microscan3
{
public:
  /*!
   *  Typedef for function which has to be passed to this class. This enables the use of
   *  functions from the calling class.
   */
  typedef boost::function<void(const sick::datastructure::Data&)> packetReceivedCallbackFunction;

  /*!
   * \brief Constructor of the Microscan3 class
   * \param newPacketReceivedCallbackFunction Function from the calling class, which will be
   * called when a new packet is received
   * \param settings Current settings for the sensor
   */
  Microscan3(packetReceivedCallbackFunction newPacketReceivedCallbackFunction,
             sick::datastructure::CommSettings settings);

  /*!
   * \brief Destructor
   */
  virtual ~Microscan3();

  /*!
   * \brief Start the connection to the sensor and enables output
   * \return If the setup was correct
   */

  bool run();

  /*!
   * \brief Changes the internal settings of the sensor
   * \param settings New set of settign to pass to the sensor
   */
  void changeSensorSettings(const sick::datastructure::CommSettings &settings);

  /*!
   * \brief Requests the typecode of the sensor.
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param type_code Returned typecode.
   */
  void requestTypeCode(const sick::datastructure::CommSettings &settings, sick::datastructure::TypeCode &type_code);
private:
  packetReceivedCallbackFunction m_newPacketReceivedCallbackFunction;

  std::shared_ptr<boost::asio::io_service> m_io_service_ptr;
  std::shared_ptr<boost::asio::io_service::work> m_io_work_ptr;
  std::shared_ptr<sick::communication::AsyncUDPClient> m_async_udp_client_ptr;
  std::shared_ptr<sick::communication::AsyncTCPClient> m_async_tcp_client_ptr;
  boost::scoped_ptr<boost::thread> m_udp_client_thread_ptr;

  std::shared_ptr<sick::cola2::Cola2Session> m_session_ptr;

  std::shared_ptr<sick::data_processing::UDPPacketMerger> m_packet_merger_ptr;

  std::string m_device_name;

  void processUDPPacket(const datastructure::PacketBuffer& buffer);
  bool UDPClientThread();
  void processTCPPacket(const sick::datastructure::PacketBuffer& buffer);
  void startTCPConnection(const sick::datastructure::CommSettings& settings);
  void changeCommSettingsinColaSession(const datastructure::CommSettings& settings);
  void stopTCPConnection();
  void requestTypeCodeinColaSession( sick::datastructure::TypeCode &type_code);
};

} // namespace sick

#endif
