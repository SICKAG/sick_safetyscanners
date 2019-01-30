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
 * \file SickSafetyscanners.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_SICKSAFETYSCANNERS_H
#define SICK_SAFETYSCANNERS_SICKSAFETYSCANNERS_H

#include <ros/ros.h>

#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <string>
#include <vector>

#include <sick_safetyscanners/communication/AsyncTCPClient.h>
#include <sick_safetyscanners/communication/AsyncUDPClient.h>
#include <sick_safetyscanners/data_processing/ParseData.h>
#include <sick_safetyscanners/data_processing/UDPPacketMerger.h>
#include <sick_safetyscanners/datastructure/CommSettings.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/cola2/ChangeCommSettingsCommand.h>
#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/DeviceNameVariableCommand.h>
#include <sick_safetyscanners/cola2/FieldGeometryVariableCommand.h>
#include <sick_safetyscanners/cola2/FieldHeaderVariableCommand.h>
#include <sick_safetyscanners/cola2/MeasurementCurrentConfigVariableCommand.h>
#include <sick_safetyscanners/cola2/MeasurementPersistentConfigVariableCommand.h>
#include <sick_safetyscanners/cola2/MonitoringCaseTableHeaderVariableCommand.h>
#include <sick_safetyscanners/cola2/MonitoringCaseVariableCommand.h>
#include <sick_safetyscanners/cola2/TypeCodeVariableCommand.h>

namespace sick {

/*!
 * \brief Class managing the algorithmic part of the package.
 */
class SickSafetyscanners
{
public:
  /*!
   *  Typedef for function which has to be passed to this class. This enables the use of
   *  functions from the calling class.
   */
  typedef boost::function<void(const sick::datastructure::Data&)> packetReceivedCallbackFunction;

  /*!
   * \brief Constructor of the SickSafetyscanners class.
   * \param newPacketReceivedCallbackFunction Function from the calling class, which will be
   * called when a new packet is received.
   * \param settings Current settings for the sensor.
   */
  SickSafetyscanners(packetReceivedCallbackFunction newPacketReceivedCallbackFunction,
                     sick::datastructure::CommSettings* settings);

  /*!
   * \brief Destructor
   */
  virtual ~SickSafetyscanners();

  /*!
   * \brief Start the connection to the sensor and enables output.
   * \return If the setup was correct.
   */
  bool run();

  /*!
   * \brief Changes the internal settings of the sensor.
   * \param settings New set of settings to pass to the sensor.
   */
  void changeSensorSettings(const sick::datastructure::CommSettings& settings);

  /*!
   * \brief Requests the typecode of the sensor.
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param type_code Returned typecode.
   */
  void requestTypeCode(const sick::datastructure::CommSettings& settings,
                       sick::datastructure::TypeCode& type_code);

  /*!
   * \brief Requests data of the protective and warning fields from the sensor.
   *
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param field_data Returned field data.
   */
  void requestFieldData(const sick::datastructure::CommSettings& settings,
                        std::vector<sick::datastructure::FieldData>& field_data);

  /*!
   * \brief Requests the name of the device from the sensor.
   *
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param device_name Returned device name.
   */
  void requestDeviceName(const sick::datastructure::CommSettings& settings,
                         std::string& device_name);

  /*!
   * \brief Requests the monitoring cases from the sensor.
   *
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param monitoring_cases Returned monitoring cases.
   */
  void
  requestMonitoringCases(const sick::datastructure::CommSettings& settings,
                         std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases);


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
  int m_active_case_number;

  void processUDPPacket(const sick::datastructure::PacketBuffer& buffer);
  bool UDPClientThread();
  void processTCPPacket(const sick::datastructure::PacketBuffer& buffer);
  void startTCPConnection(const sick::datastructure::CommSettings& settings);
  void changeCommSettingsInColaSession(const datastructure::CommSettings& settings);
  void stopTCPConnection();
  void requestTypeCodeInColaSession(sick::datastructure::TypeCode& type_code);
  void requestFieldDataInColaSession(std::vector<sick::datastructure::FieldData>& fields);
  void requestDeviceNameInColaSession(std::string& device_name);
  void requestMonitoringCaseDataInColaSession(
  std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases);
};

} // namespace sick


#endif // SICK_SAFETYSCANNERS_SICKSAFETYSCANNERS_H
