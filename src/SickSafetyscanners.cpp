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
 * \file SickSafetyscanners.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#include "sick_safetyscanners/SickSafetyscanners.h"

namespace sick {

SickSafetyscanners::SickSafetyscanners(
  const packetReceivedCallbackFunction& newPacketReceivedCallbackFunction,
  sick::datastructure::CommSettings* settings)
  : m_newPacketReceivedCallbackFunction(newPacketReceivedCallbackFunction)
{
  ROS_INFO("Starting SickSafetyscanners");
  m_io_service_ptr       = std::make_shared<boost::asio::io_service>();
  m_async_udp_client_ptr = std::make_shared<sick::communication::AsyncUDPClient>(
    boost::bind(&SickSafetyscanners::processUDPPacket, this, _1),
    boost::ref(*m_io_service_ptr),
    settings->getHostUdpPort());
  settings->setHostUdpPort(
    m_async_udp_client_ptr
      ->getLocalPort()); // Store which port was used, needed for data request from the laser
  m_packet_merger_ptr = std::make_shared<sick::data_processing::UDPPacketMerger>();
  ROS_INFO("Started SickSafetyscanners");
}

SickSafetyscanners::~SickSafetyscanners()
{
  m_io_service_ptr->stop();
  m_udp_client_thread_ptr->join();
  m_udp_client_thread_ptr.reset();
}

bool SickSafetyscanners::run()
{
  m_udp_client_thread_ptr.reset(
    new boost::thread(boost::bind(&SickSafetyscanners::udpClientThread, this)));

  m_async_udp_client_ptr->runService();
  return true;
}

bool SickSafetyscanners::udpClientThread()
{
  ROS_INFO("Enter io thread");
  m_io_work_ptr = std::make_shared<boost::asio::io_service::work>(boost::ref(*m_io_service_ptr));
  m_io_service_ptr->run();
  ROS_INFO("Exit io thread");
  return true;
}


void SickSafetyscanners::processTCPPacket(const sick::datastructure::PacketBuffer& buffer)
{
  // Not needed for current functionality, inplace for possible future developments
}

void SickSafetyscanners::changeSensorSettings(const datastructure::CommSettings& settings)
{
  startTCPConnection(settings);
  changeCommSettingsInColaSession(settings);
  stopTCPConnection();
}

void SickSafetyscanners::FindSensor(const datastructure::CommSettings& settings,
                                    uint16_t blink_time)
{
  startTCPConnection(settings);
  FindSensorInColaSession(blink_time);
  stopTCPConnection();
}

void SickSafetyscanners::requestTypeCode(const datastructure::CommSettings& settings,
                                         sick::datastructure::TypeCode& type_code)
{
  startTCPConnection(settings);
  requestTypeCodeInColaSession(type_code);
  stopTCPConnection();
}

void SickSafetyscanners::requestApplicationName(
  const datastructure::CommSettings& settings,
  sick::datastructure::ApplicationName& application_name)
{
  startTCPConnection(settings);
  requestApplicationNameInColaSession(application_name);
  stopTCPConnection();
}
void SickSafetyscanners::requestFieldData(const datastructure::CommSettings& settings,
                                          std::vector<sick::datastructure::FieldData>& field_data)
{
  startTCPConnection(settings);
  requestFieldDataInColaSession(field_data);
  stopTCPConnection();
}

void SickSafetyscanners::requestMonitoringCases(
  const datastructure::CommSettings& settings,
  std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases)
{
  startTCPConnection(settings);
  requestMonitoringCaseDataInColaSession(monitoring_cases);
  stopTCPConnection();
}

void SickSafetyscanners::requestDeviceName(const datastructure::CommSettings& settings,
                                           datastructure::DeviceName& device_name)
{
  startTCPConnection(settings);
  requestDeviceNameInColaSession(device_name);
  stopTCPConnection();
}

void SickSafetyscanners::requestSerialNumber(const datastructure::CommSettings& settings,
                                             datastructure::SerialNumber& serial_number)
{
  startTCPConnection(settings);
  requestSerialNumberInColaSession(serial_number);
  stopTCPConnection();
}

void SickSafetyscanners::requestOrderNumber(const datastructure::CommSettings& settings,
                                            datastructure::OrderNumber& order_number)
{
  startTCPConnection(settings);
  requestOrderNumberInColaSession(order_number);
  stopTCPConnection();
}

void SickSafetyscanners::requestProjectName(const datastructure::CommSettings& settings,
                                            datastructure::ProjectName& project_name)
{
  startTCPConnection(settings);
  requestProjectNameInColaSession(project_name);
  stopTCPConnection();
}

void SickSafetyscanners::requestUserName(const datastructure::CommSettings& settings,
                                         datastructure::UserName& user_name)
{
  startTCPConnection(settings);
  requestUserNameInColaSession(user_name);
  stopTCPConnection();
}
void SickSafetyscanners::requestFirmwareVersion(const datastructure::CommSettings& settings,
                                                datastructure::FirmwareVersion& firmware_version)
{
  startTCPConnection(settings);
  requestFirmwareVersionInColaSession(firmware_version);
  stopTCPConnection();
}

void SickSafetyscanners::requestPersistentConfig(const datastructure::CommSettings& settings,
                                                 sick::datastructure::ConfigData& config_data)
{
  startTCPConnection(settings);
  requestPersistentConfigInColaSession(config_data);
  stopTCPConnection();
}

void SickSafetyscanners::requestConfigMetadata(const datastructure::CommSettings& settings,
                                               sick::datastructure::ConfigMetadata& config_metadata)
{
  startTCPConnection(settings);
  requestConfigMetadataInColaSession(config_metadata);
  stopTCPConnection();
}

void SickSafetyscanners::requestStatusOverview(const datastructure::CommSettings& settings,
                                               sick::datastructure::StatusOverview& status_overview)
{
  startTCPConnection(settings);
  requestStatusOverviewInColaSession(status_overview);
  stopTCPConnection();
}

void SickSafetyscanners::requestDeviceStatus(const datastructure::CommSettings& settings,
                                             sick::datastructure::DeviceStatus& device_status)
{
  startTCPConnection(settings);
  requestDeviceStatusInColaSession(device_status);
  stopTCPConnection();
}

void SickSafetyscanners::requestRequiredUserAction(
  const datastructure::CommSettings& settings,
  sick::datastructure::RequiredUserAction& required_user_action)
{
  startTCPConnection(settings);
  requestRequiredUserActionInColaSession(required_user_action);
  stopTCPConnection();
}

void SickSafetyscanners::startTCPConnection(const sick::datastructure::CommSettings& settings)
{
  std::shared_ptr<sick::communication::AsyncTCPClient> async_tcp_client =
    std::make_shared<sick::communication::AsyncTCPClient>(
      boost::bind(&SickSafetyscanners::processTCPPacket, this, _1),
      boost::ref(*m_io_service_ptr),
      settings.getSensorIp(),
      settings.getSensorTcpPort());
  async_tcp_client->doConnect();

  m_session_ptr.reset();
  m_session_ptr = std::make_shared<sick::cola2::Cola2Session>(async_tcp_client);

  m_session_ptr->open();
}

void SickSafetyscanners::changeCommSettingsInColaSession(
  const datastructure::CommSettings& settings)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ChangeCommSettingsCommand>(boost::ref(*m_session_ptr), settings);
  m_session_ptr->executeCommand(command_ptr);
}

void SickSafetyscanners::requestFieldDataInColaSession(
  std::vector<sick::datastructure::FieldData>& fields)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr;

  for (int i = 0; i < 128; i++)
  {
    sick::datastructure::FieldData field_data;

    command_ptr = std::make_shared<sick::cola2::FieldHeaderVariableCommand>(
      boost::ref(*m_session_ptr), field_data, i);
    m_session_ptr->executeCommand(command_ptr);

    if (field_data.getIsValid())
    {
      command_ptr = std::make_shared<sick::cola2::FieldGeometryVariableCommand>(
        boost::ref(*m_session_ptr), field_data, i);
      m_session_ptr->executeCommand(command_ptr);

      fields.push_back(field_data);
    }
    else if (i > 0) // index 0 is reserved for contour data
    {
      break; // skip other requests after first invalid
    }
  }
}

void SickSafetyscanners::requestMonitoringCaseDataInColaSession(
  std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr;
  for (int i = 0; i < 254; i++)
  {
    sick::datastructure::MonitoringCaseData monitoring_case_data;

    command_ptr = std::make_shared<sick::cola2::MonitoringCaseVariableCommand>(
      boost::ref(*m_session_ptr), monitoring_case_data, i);
    m_session_ptr->executeCommand(command_ptr);
    if (monitoring_case_data.getIsValid())
    {
      monitoring_cases.push_back(monitoring_case_data);
    }
    else
    {
      break; // skip other requests after first invalid
    }
  }
}

void SickSafetyscanners::FindSensorInColaSession(uint16_t blink_time)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::FindMeCommand>(boost::ref(*m_session_ptr), blink_time);
  m_session_ptr->executeCommand(command_ptr);
}

void SickSafetyscanners::requestDeviceNameInColaSession(datastructure::DeviceName& device_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::DeviceNameVariableCommand>(boost::ref(*m_session_ptr),
                                                             device_name);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Device name: %s", device_name.getDeviceName().c_str());
}


void SickSafetyscanners::requestApplicationNameInColaSession(
  datastructure::ApplicationName& application_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ApplicationNameVariableCommand>(boost::ref(*m_session_ptr),
                                                                  application_name);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Application name: %s", application_name.getApplicationName().c_str());
}

void SickSafetyscanners::requestSerialNumberInColaSession(
  datastructure::SerialNumber& serial_number)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::SerialNumberVariableCommand>(boost::ref(*m_session_ptr),
                                                               serial_number);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Serial Number: %s", serial_number.getSerialNumber().c_str());
}

void SickSafetyscanners::requestFirmwareVersionInColaSession(
  datastructure::FirmwareVersion& firmware_version)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::FirmwareVersionVariableCommand>(boost::ref(*m_session_ptr),
                                                                  firmware_version);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Firmware Version: %s", firmware_version.getFirmwareVersion().c_str());
}

void SickSafetyscanners::requestTypeCodeInColaSession(sick::datastructure::TypeCode& type_code)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::TypeCodeVariableCommand>(boost::ref(*m_session_ptr), type_code);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Type Code: %s", type_code.getTypeCode().c_str());
}

void SickSafetyscanners::requestOrderNumberInColaSession(
  sick::datastructure::OrderNumber& order_number)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::OrderNumberVariableCommand>(boost::ref(*m_session_ptr),
                                                              order_number);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Order Number: %s", order_number.getOrderNumber().c_str());
}

void SickSafetyscanners::requestProjectNameInColaSession(
  sick::datastructure::ProjectName& project_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ProjectNameVariableCommand>(boost::ref(*m_session_ptr),
                                                              project_name);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Project Name: %s", project_name.getProjectName().c_str());
}

void SickSafetyscanners::requestUserNameInColaSession(sick::datastructure::UserName& user_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::UserNameVariableCommand>(boost::ref(*m_session_ptr), user_name);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("User Name: %s", user_name.getUserName().c_str());
}

void SickSafetyscanners::requestConfigMetadataInColaSession(
  sick::datastructure::ConfigMetadata& config_metadata)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ConfigMetadataVariableCommand>(boost::ref(*m_session_ptr),
                                                                 config_metadata);
  m_session_ptr->executeCommand(command_ptr);
}

void SickSafetyscanners::requestStatusOverviewInColaSession(
  sick::datastructure::StatusOverview& status_overview)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::StatusOverviewVariableCommand>(boost::ref(*m_session_ptr),
                                                                 status_overview);
  m_session_ptr->executeCommand(command_ptr);
}

void SickSafetyscanners::requestDeviceStatusInColaSession(
  sick::datastructure::DeviceStatus& device_status)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::DeviceStatusVariableCommand>(boost::ref(*m_session_ptr),
                                                               device_status);
  m_session_ptr->executeCommand(command_ptr);
}

void SickSafetyscanners::requestRequiredUserActionInColaSession(
  sick::datastructure::RequiredUserAction& required_user_action)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::RequiredUserActionVariableCommand>(boost::ref(*m_session_ptr),
                                                                     required_user_action);
  m_session_ptr->executeCommand(command_ptr);
}

void SickSafetyscanners::requestPersistentConfigInColaSession(
  sick::datastructure::ConfigData& config_data)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::MeasurementPersistentConfigVariableCommand>(
      boost::ref(*m_session_ptr), config_data);
  m_session_ptr->executeCommand(command_ptr);
}

void SickSafetyscanners::stopTCPConnection()
{
  m_session_ptr->close();
  m_session_ptr->doDisconnect();
}


void SickSafetyscanners::processUDPPacket(const sick::datastructure::PacketBuffer& buffer)
{
  if (m_packet_merger_ptr->addUDPPacket(buffer))
  {
    sick::datastructure::PacketBuffer deployed_buffer =
      m_packet_merger_ptr->getDeployedPacketBuffer();
    sick::data_processing::ParseData data_parser;
    sick::datastructure::Data data = data_parser.parseUDPSequence(deployed_buffer);

    m_newPacketReceivedCallbackFunction(data);
  }
}

} // namespace sick
