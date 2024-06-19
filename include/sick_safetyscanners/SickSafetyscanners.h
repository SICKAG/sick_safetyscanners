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
#include <sick_safetyscanners/datastructure/ConfigData.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/cola2/ApplicationNameVariableCommand.h>
#include <sick_safetyscanners/cola2/ChangeCommSettingsCommand.h>
#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/ConfigMetadataVariableCommand.h>
#include <sick_safetyscanners/cola2/DeviceNameVariableCommand.h>
#include <sick_safetyscanners/cola2/DeviceStatusVariableCommand.h>
#include <sick_safetyscanners/cola2/FieldGeometryVariableCommand.h>
#include <sick_safetyscanners/cola2/FieldHeaderVariableCommand.h>
#include <sick_safetyscanners/cola2/FindMeCommand.h>
#include <sick_safetyscanners/cola2/FirmwareVersionVariableCommand.h>
#include <sick_safetyscanners/cola2/MeasurementCurrentConfigVariableCommand.h>
#include <sick_safetyscanners/cola2/MeasurementPersistentConfigVariableCommand.h>
#include <sick_safetyscanners/cola2/MonitoringCaseTableHeaderVariableCommand.h>
#include <sick_safetyscanners/cola2/MonitoringCaseVariableCommand.h>
#include <sick_safetyscanners/cola2/OrderNumberVariableCommand.h>
#include <sick_safetyscanners/cola2/ProjectNameVariableCommand.h>
#include <sick_safetyscanners/cola2/RequiredUserActionVariableCommand.h>
#include <sick_safetyscanners/cola2/SerialNumberVariableCommand.h>
#include <sick_safetyscanners/cola2/StatusOverviewVariableCommand.h>
#include <sick_safetyscanners/cola2/TypeCodeVariableCommand.h>
#include <sick_safetyscanners/cola2/UserNameVariableCommand.h>

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
  SickSafetyscanners(const packetReceivedCallbackFunction& newPacketReceivedCallbackFunction,
                     sick::datastructure::CommSettings* settings,
                     boost::asio::ip::address_v4 interface_ip);

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
  bool changeSensorSettings(const sick::datastructure::CommSettings& settings);

  /*!
   * \brief Requests the typecode of the sensor.
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param type_code Returned typecode.
   */
  bool requestTypeCode(const sick::datastructure::CommSettings& settings,
                       sick::datastructure::TypeCode& type_code);

  bool requestApplicationName(const sick::datastructure::CommSettings& settings,
                                sick::datastructure::ApplicationName& application_name);

  /*!
   * \brief Requests the config meta data of the sensor.
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param config_metadata Returned config meta data.
   */
  bool requestConfigMetadata(const datastructure::CommSettings& settings,
                             datastructure::ConfigMetadata& config_metadata);

  /*!
   * \brief Requests the firmware version of the sensor.
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param firmware_version Returned firmware version.
   */
  bool requestFirmwareVersion(const sick::datastructure::CommSettings& settings,
                              sick::datastructure::FirmwareVersion& firmware_version);
  bool requestSerialNumber(const sick::datastructure::CommSettings& settings,
                           sick::datastructure::SerialNumber& serial_number);

  bool requestOrderNumber(const datastructure::CommSettings& settings,
                          datastructure::OrderNumber& order_number);
  bool requestProjectName(const datastructure::CommSettings& settings,
                          datastructure::ProjectName& project_name);
  bool requestUserName(const datastructure::CommSettings& settings,
                       datastructure::UserName& user_name);

  bool requestStatusOverview(const datastructure::CommSettings& settings,
                             datastructure::StatusOverview& status_overview);
  bool requestDeviceStatus(const datastructure::CommSettings& settings,
                           datastructure::DeviceStatus& device_status);
  bool requestRequiredUserAction(const datastructure::CommSettings& settings,
                                 datastructure::RequiredUserAction& required_user_action);
  bool FindSensor(const datastructure::CommSettings& settings, uint16_t blink_time);
  /*!
   * \brief Requests data of the protective and warning fields from the sensor.
   *
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param field_data Returned field data.
   */
  bool requestFieldData(const sick::datastructure::CommSettings& settings,
                        std::vector<sick::datastructure::FieldData>& field_data);

  /*!
   * \brief Requests the name of the device from the sensor.
   *
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param device_name Returned device name.
   */
  bool requestDeviceName(const sick::datastructure::CommSettings& settings,
                         datastructure::DeviceName& device_name);

  /*!
   * \brief Requests the persistent configuration from the sensor.
   *
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param config_data Returned persistent configuration data.
   */
  bool requestPersistentConfig(const datastructure::CommSettings& settings,
                               sick::datastructure::ConfigData& config_data);
  /*!
   * \brief Requests the monitoring cases from the sensor.
   *
   * \param settings Settings containing information to establish a connection to the sensor.
   * \param monitoring_cases Returned monitoring cases.
   */
  bool
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

  void processUDPPacket(const sick::datastructure::PacketBuffer& buffer);
  bool udpClientThread();
  void processTCPPacket(const sick::datastructure::PacketBuffer& buffer);
  bool startTCPConnection(const sick::datastructure::CommSettings& settings);
  bool changeCommSettingsInColaSession(const datastructure::CommSettings& settings);
  bool stopTCPConnection();
  bool requestTypeCodeInColaSession(sick::datastructure::TypeCode& type_code);
  bool requestFieldDataInColaSession(std::vector<sick::datastructure::FieldData>& fields);
  bool requestDeviceNameInColaSession(datastructure::DeviceName& device_name);
  bool requestApplicationNameInColaSession(sick::datastructure::ApplicationName& application_name);
  bool requestSerialNumberInColaSession(sick::datastructure::SerialNumber& serial_number);
  bool requestOrderNumberInColaSession(sick::datastructure::OrderNumber& order_number);
  bool requestProjectNameInColaSession(sick::datastructure::ProjectName& project_name);
  bool requestUserNameInColaSession(sick::datastructure::UserName& user_name);
  bool requestFirmwareVersionInColaSession(sick::datastructure::FirmwareVersion& firmware_version);
  bool requestPersistentConfigInColaSession(sick::datastructure::ConfigData& config_data);
  bool requestConfigMetadataInColaSession(sick::datastructure::ConfigMetadata& config_metadata);
  bool requestStatusOverviewInColaSession(sick::datastructure::StatusOverview& status_overview);
  bool requestDeviceStatusInColaSession(sick::datastructure::DeviceStatus& device_status);
  bool requestRequiredUserActionInColaSession(
    sick::datastructure::RequiredUserAction& required_user_action);
  bool requestMonitoringCaseDataInColaSession(
    std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases);
  bool FindSensorInColaSession(uint16_t blink_time);
};

} // namespace sick


#endif // SICK_SAFETYSCANNERS_SICKSAFETYSCANNERS_H
