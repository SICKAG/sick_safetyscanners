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
  sick::datastructure::CommSettings* settings,
  boost::asio::ip::address_v4 interface_ip)
  : m_newPacketReceivedCallbackFunction(newPacketReceivedCallbackFunction)
{
  ROS_INFO("Starting SickSafetyscanners");
  m_io_service_ptr = std::make_shared<boost::asio::io_service>();
  if (settings->getHostIp().is_multicast())
  {
    ROS_INFO("Multicast Host Ip configured");
    m_async_udp_client_ptr = std::make_shared<sick::communication::AsyncUDPClient>(
      boost::bind(&SickSafetyscanners::processUDPPacket, this, _1),
      boost::ref(*m_io_service_ptr),
      settings->getHostIp(),
      // boost::asio::ip::address_v4::from_string("192.168.1.9"),
      interface_ip,
      settings->getHostUdpPort());
  }
  else
  {
    m_async_udp_client_ptr = std::make_shared<sick::communication::AsyncUDPClient>(
      boost::bind(&SickSafetyscanners::processUDPPacket, this, _1),
      boost::ref(*m_io_service_ptr),
      settings->getHostUdpPort());
  }
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

bool SickSafetyscanners::changeSensorSettings(const datastructure::CommSettings& settings)
{
  bool status = false;

  status = startTCPConnection(settings);
  if (true == status)
  {
    changeCommSettingsInColaSession(settings);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::FindSensor(const datastructure::CommSettings& settings,
                                    uint16_t blink_time)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    FindSensorInColaSession(blink_time);
    stopTCPConnection();
  }
  
  return status;
}

bool SickSafetyscanners::requestTypeCode(const datastructure::CommSettings& settings,
                                         sick::datastructure::TypeCode& type_code)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestTypeCodeInColaSession(type_code);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestApplicationName(
  const datastructure::CommSettings& settings,
  sick::datastructure::ApplicationName& application_name)
{

  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestApplicationNameInColaSession(application_name);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestFieldData(const datastructure::CommSettings& settings,
                                          std::vector<sick::datastructure::FieldData>& field_data)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestFieldDataInColaSession(field_data);
    stopTCPConnection();
  }
  
  return status;
}

bool SickSafetyscanners::requestMonitoringCases(
  const datastructure::CommSettings& settings,
  std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestMonitoringCaseDataInColaSession(monitoring_cases);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestDeviceName(const datastructure::CommSettings& settings,
                                           datastructure::DeviceName& device_name)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestDeviceNameInColaSession(device_name);
    stopTCPConnection();
  }
  
  return status;
}

bool SickSafetyscanners::requestSerialNumber(const datastructure::CommSettings& settings,
                                             datastructure::SerialNumber& serial_number)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestSerialNumberInColaSession(serial_number);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestOrderNumber(const datastructure::CommSettings& settings,
                                            datastructure::OrderNumber& order_number)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestOrderNumberInColaSession(order_number);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestProjectName(const datastructure::CommSettings& settings,
                                            datastructure::ProjectName& project_name)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestProjectNameInColaSession(project_name);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestUserName(const datastructure::CommSettings& settings,
                                         datastructure::UserName& user_name)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestUserNameInColaSession(user_name);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestFirmwareVersion(const datastructure::CommSettings& settings,
                                                datastructure::FirmwareVersion& firmware_version)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestFirmwareVersionInColaSession(firmware_version);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestPersistentConfig(const datastructure::CommSettings& settings,
                                                 sick::datastructure::ConfigData& config_data)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
   requestPersistentConfigInColaSession(config_data);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestConfigMetadata(const datastructure::CommSettings& settings,
                                               sick::datastructure::ConfigMetadata& config_metadata)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestConfigMetadataInColaSession(config_metadata);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestStatusOverview(const datastructure::CommSettings& settings,
                                               sick::datastructure::StatusOverview& status_overview)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestStatusOverviewInColaSession(status_overview);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestDeviceStatus(const datastructure::CommSettings& settings,
                                             sick::datastructure::DeviceStatus& device_status)
{
  bool status = false;
  status = startTCPConnection(settings);

  if (true == status)
  {
    requestDeviceStatusInColaSession(device_status);
    stopTCPConnection();
  }

  return status;
}

bool SickSafetyscanners::requestRequiredUserAction(
  const datastructure::CommSettings& settings,
  sick::datastructure::RequiredUserAction& required_user_action)
{
  bool status = false;
  status = startTCPConnection(settings);
  
  if (true == status)
  {
    requestRequiredUserActionInColaSession(required_user_action);
    stopTCPConnection();
  }
  
  return status;
}

bool SickSafetyscanners::startTCPConnection(const sick::datastructure::CommSettings& settings)
{
  bool status = false;
  std::shared_ptr<sick::communication::AsyncTCPClient> async_tcp_client =
    std::make_shared<sick::communication::AsyncTCPClient>(
      boost::bind(&SickSafetyscanners::processTCPPacket, this, _1),
      boost::ref(*m_io_service_ptr),
      settings.getSensorIp(),
      settings.getSensorTcpPort());
  
  status = async_tcp_client->doConnect();

  if (true == status)
  {
    m_session_ptr.reset();
    m_session_ptr = std::make_shared<sick::cola2::Cola2Session>(async_tcp_client);
    
    status = m_session_ptr->open();
  }

  return status;
}

bool SickSafetyscanners::changeCommSettingsInColaSession(
  const datastructure::CommSettings& settings)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ChangeCommSettingsCommand>(boost::ref(*m_session_ptr), settings);
  return m_session_ptr->executeCommand(command_ptr);
}

bool SickSafetyscanners::requestFieldDataInColaSession(
  std::vector<sick::datastructure::FieldData>& fields)
{
  bool status = false;

  sick::cola2::Cola2Session::CommandPtr command_ptr;

  for (int i = 0; i < 128; i++)
  {
    sick::datastructure::FieldData field_data;

    command_ptr = std::make_shared<sick::cola2::FieldHeaderVariableCommand>(
      boost::ref(*m_session_ptr), field_data, i);
    
    status = m_session_ptr->executeCommand(command_ptr);

    if (field_data.getIsValid())
    {
      command_ptr = std::make_shared<sick::cola2::FieldGeometryVariableCommand>(
        boost::ref(*m_session_ptr), field_data, i);
      status = m_session_ptr->executeCommand(command_ptr);

      fields.push_back(field_data);
    }
    else if (i > 0) // index 0 is reserved for contour data
    {
      break; // skip other requests after first invalid
    }
  }

  return status;
}

bool SickSafetyscanners::requestMonitoringCaseDataInColaSession(
  std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases)
{
  bool status = false;

  sick::cola2::Cola2Session::CommandPtr command_ptr;
  for (int i = 0; i < 254; i++)
  {
    sick::datastructure::MonitoringCaseData monitoring_case_data;

    command_ptr = std::make_shared<sick::cola2::MonitoringCaseVariableCommand>(
      boost::ref(*m_session_ptr), monitoring_case_data, i);
    status = m_session_ptr->executeCommand(command_ptr);
    if (monitoring_case_data.getIsValid())
    {
      monitoring_cases.push_back(monitoring_case_data);
    }
    else
    {
      break; // skip other requests after first invalid
    }
  }

  return status;
}

bool SickSafetyscanners::FindSensorInColaSession(uint16_t blink_time)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::FindMeCommand>(boost::ref(*m_session_ptr), blink_time);
  return m_session_ptr->executeCommand(command_ptr);
}

bool SickSafetyscanners::requestDeviceNameInColaSession(datastructure::DeviceName& device_name)
{
  bool status = false;
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::DeviceNameVariableCommand>(boost::ref(*m_session_ptr),
                                                             device_name);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Device name: %s", device_name.getDeviceName().c_str());

  return status;
}


bool SickSafetyscanners::requestApplicationNameInColaSession(
  datastructure::ApplicationName& application_name)
{
  bool status = false;
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ApplicationNameVariableCommand>(boost::ref(*m_session_ptr),
                                                                  application_name);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Application name: %s", application_name.getApplicationName().c_str());
  
  return status;
}

bool SickSafetyscanners::requestSerialNumberInColaSession(
  datastructure::SerialNumber& serial_number)
{
  bool status = false;
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::SerialNumberVariableCommand>(boost::ref(*m_session_ptr),
                                                               serial_number);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Serial Number: %s", serial_number.getSerialNumber().c_str());
  
  return status;
}

bool SickSafetyscanners::requestFirmwareVersionInColaSession(
  datastructure::FirmwareVersion& firmware_version)
{
  bool status = false;

  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::FirmwareVersionVariableCommand>(boost::ref(*m_session_ptr),
                                                                  firmware_version);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Firmware Version: %s", firmware_version.getFirmwareVersion().c_str());

  return status;
}

bool SickSafetyscanners::requestTypeCodeInColaSession(sick::datastructure::TypeCode& type_code)
{
  bool status = false;
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::TypeCodeVariableCommand>(boost::ref(*m_session_ptr), type_code);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Type Code: %s", type_code.getTypeCode().c_str());
  
  return status;
}

bool SickSafetyscanners::requestOrderNumberInColaSession(
  sick::datastructure::OrderNumber& order_number)
{
  bool status = false;
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::OrderNumberVariableCommand>(boost::ref(*m_session_ptr),
                                                              order_number);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Order Number: %s", order_number.getOrderNumber().c_str());
  return status;
}

bool SickSafetyscanners::requestProjectNameInColaSession(
  sick::datastructure::ProjectName& project_name)
{
  bool status = false;
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ProjectNameVariableCommand>(boost::ref(*m_session_ptr),
                                                              project_name);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Project Name: %s", project_name.getProjectName().c_str());
  return status;
}

bool SickSafetyscanners::requestUserNameInColaSession(sick::datastructure::UserName& user_name)
{
  bool status = false;
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::UserNameVariableCommand>(boost::ref(*m_session_ptr), user_name);
  status = m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("User Name: %s", user_name.getUserName().c_str());
  return status;
}

bool SickSafetyscanners::requestConfigMetadataInColaSession(
  sick::datastructure::ConfigMetadata& config_metadata)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ConfigMetadataVariableCommand>(boost::ref(*m_session_ptr),
                                                                 config_metadata);
  return m_session_ptr->executeCommand(command_ptr);
}

bool SickSafetyscanners::requestStatusOverviewInColaSession(
  sick::datastructure::StatusOverview& status_overview)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::StatusOverviewVariableCommand>(boost::ref(*m_session_ptr),
                                                                 status_overview);
  return m_session_ptr->executeCommand(command_ptr);
}

bool SickSafetyscanners::requestDeviceStatusInColaSession(
  sick::datastructure::DeviceStatus& device_status)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::DeviceStatusVariableCommand>(boost::ref(*m_session_ptr),
                                                               device_status);
  return m_session_ptr->executeCommand(command_ptr);
}

bool SickSafetyscanners::requestRequiredUserActionInColaSession(
  sick::datastructure::RequiredUserAction& required_user_action)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::RequiredUserActionVariableCommand>(boost::ref(*m_session_ptr),
                                                                     required_user_action);
 return m_session_ptr->executeCommand(command_ptr);
}

bool SickSafetyscanners::requestPersistentConfigInColaSession(
  sick::datastructure::ConfigData& config_data)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::MeasurementPersistentConfigVariableCommand>(
      boost::ref(*m_session_ptr), config_data);
  return m_session_ptr->executeCommand(command_ptr);
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
