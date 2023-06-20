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
  if (!startTCPConnection(settings))
    return false;

  if (!changeCommSettingsInColaSession(settings))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::FindSensor(const datastructure::CommSettings& settings,
                                    uint16_t blink_time)
{
  if (!startTCPConnection(settings))
    return false;

  if (!FindSensorInColaSession(blink_time))
    return false;
  
  return stopTCPConnection();
}

bool SickSafetyscanners::requestTypeCode(const datastructure::CommSettings& settings,
                                         sick::datastructure::TypeCode& type_code)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestTypeCodeInColaSession(type_code))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestApplicationName(
  const datastructure::CommSettings& settings,
  sick::datastructure::ApplicationName& application_name)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestApplicationNameInColaSession(application_name))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestFieldData(const datastructure::CommSettings& settings,
                                          std::vector<sick::datastructure::FieldData>& field_data)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestFieldDataInColaSession(field_data))
    return false;
  
  return stopTCPConnection();
}

bool SickSafetyscanners::requestMonitoringCases(
  const datastructure::CommSettings& settings,
  std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestMonitoringCaseDataInColaSession(monitoring_cases))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestDeviceName(const datastructure::CommSettings& settings,
                                           datastructure::DeviceName& device_name)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestDeviceNameInColaSession(device_name))
    return false;
  
  return stopTCPConnection();
}

bool SickSafetyscanners::requestSerialNumber(const datastructure::CommSettings& settings,
                                             datastructure::SerialNumber& serial_number)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestSerialNumberInColaSession(serial_number))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestOrderNumber(const datastructure::CommSettings& settings,
                                            datastructure::OrderNumber& order_number)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestOrderNumberInColaSession(order_number))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestProjectName(const datastructure::CommSettings& settings,
                                            datastructure::ProjectName& project_name)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestProjectNameInColaSession(project_name))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestUserName(const datastructure::CommSettings& settings,
                                         datastructure::UserName& user_name)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestUserNameInColaSession(user_name))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestFirmwareVersion(const datastructure::CommSettings& settings,
                                                datastructure::FirmwareVersion& firmware_version)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestFirmwareVersionInColaSession(firmware_version))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestPersistentConfig(const datastructure::CommSettings& settings,
                                                 sick::datastructure::ConfigData& config_data)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestPersistentConfigInColaSession(config_data))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestConfigMetadata(const datastructure::CommSettings& settings,
                                               sick::datastructure::ConfigMetadata& config_metadata)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestConfigMetadataInColaSession(config_metadata))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestStatusOverview(const datastructure::CommSettings& settings,
                                               sick::datastructure::StatusOverview& status_overview)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestStatusOverviewInColaSession(status_overview))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestDeviceStatus(const datastructure::CommSettings& settings,
                                             sick::datastructure::DeviceStatus& device_status)
{
  if (!startTCPConnection(settings))
    return false;

  if (!requestDeviceStatusInColaSession(device_status))
    return false;

  return stopTCPConnection();
}

bool SickSafetyscanners::requestRequiredUserAction(
  const datastructure::CommSettings& settings,
  sick::datastructure::RequiredUserAction& required_user_action)
{
  if (!startTCPConnection(settings))
    return false;
  
  if (!requestRequiredUserActionInColaSession(required_user_action))
    return false;
  
  return stopTCPConnection();
}

bool SickSafetyscanners::startTCPConnection(const sick::datastructure::CommSettings& settings)
{
  std::shared_ptr<sick::communication::AsyncTCPClient> async_tcp_client =
    std::make_shared<sick::communication::AsyncTCPClient>(
      boost::bind(&SickSafetyscanners::processTCPPacket, this, _1),
      boost::ref(*m_io_service_ptr),
      settings.getSensorIp(),
      settings.getSensorTcpPort());
  
  if(!async_tcp_client->doConnect())
  {
    return false;
  }

    m_session_ptr.reset();
    m_session_ptr = std::make_shared<sick::cola2::Cola2Session>(async_tcp_client);
  return m_session_ptr->open();
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
  sick::cola2::Cola2Session::CommandPtr command_ptr;

  for (int i = 0; i < 128; i++)
  {
    sick::datastructure::FieldData field_data;

    command_ptr = std::make_shared<sick::cola2::FieldHeaderVariableCommand>(
      boost::ref(*m_session_ptr), field_data, i);
    
    /* return early if fails to execute */
    if (!m_session_ptr->executeCommand(command_ptr))
      return false;

    if (field_data.getIsValid())
    {
      command_ptr = std::make_shared<sick::cola2::FieldGeometryVariableCommand>(
        boost::ref(*m_session_ptr), field_data, i);
      
      /* return early if fails to execute */
      if (!m_session_ptr->executeCommand(command_ptr))
        return false;

      fields.push_back(field_data);
    }
    else if (i > 0) // index 0 is reserved for contour data
    {
      break; // skip other requests after first invalid
    }
  }

  return true;
}

bool SickSafetyscanners::requestMonitoringCaseDataInColaSession(
  std::vector<sick::datastructure::MonitoringCaseData>& monitoring_cases)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr;
  for (int i = 0; i < 254; i++)
  {
    sick::datastructure::MonitoringCaseData monitoring_case_data;

    command_ptr = std::make_shared<sick::cola2::MonitoringCaseVariableCommand>(
      boost::ref(*m_session_ptr), monitoring_case_data, i);

    /* Early return on failure */
    if(!m_session_ptr->executeCommand(command_ptr))
      return false;

    if (monitoring_case_data.getIsValid())
    {
      monitoring_cases.push_back(monitoring_case_data);
    }
    else
    {
      break; // skip other requests after first invalid
    }
  }

  return true;
}

bool SickSafetyscanners::FindSensorInColaSession(uint16_t blink_time)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::FindMeCommand>(boost::ref(*m_session_ptr), blink_time);
  return m_session_ptr->executeCommand(command_ptr);
}

bool SickSafetyscanners::requestDeviceNameInColaSession(datastructure::DeviceName& device_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::DeviceNameVariableCommand>(boost::ref(*m_session_ptr),
                                                             device_name);
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;

  ROS_INFO("Device name: %s", device_name.getDeviceName().c_str());
  return true;
}


bool SickSafetyscanners::requestApplicationNameInColaSession(
  datastructure::ApplicationName& application_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ApplicationNameVariableCommand>(boost::ref(*m_session_ptr),
                                                                  application_name);
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;
  
  ROS_INFO("Application name: %s", application_name.getApplicationName().c_str());
  return true;
}

bool SickSafetyscanners::requestSerialNumberInColaSession(
  datastructure::SerialNumber& serial_number)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::SerialNumberVariableCommand>(boost::ref(*m_session_ptr),
                                                               serial_number);
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;

  ROS_INFO("Serial Number: %s", serial_number.getSerialNumber().c_str());
  
  return true;
}

bool SickSafetyscanners::requestFirmwareVersionInColaSession(
  datastructure::FirmwareVersion& firmware_version)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::FirmwareVersionVariableCommand>(boost::ref(*m_session_ptr),
                                                                  firmware_version);
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;

  ROS_INFO("Firmware Version: %s", firmware_version.getFirmwareVersion().c_str());
  return true;
}

bool SickSafetyscanners::requestTypeCodeInColaSession(sick::datastructure::TypeCode& type_code)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::TypeCodeVariableCommand>(boost::ref(*m_session_ptr), type_code);
  
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;

  ROS_INFO("Type Code: %s", type_code.getTypeCode().c_str());
  return true;
}

bool SickSafetyscanners::requestOrderNumberInColaSession(
  sick::datastructure::OrderNumber& order_number)
{

  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::OrderNumberVariableCommand>(boost::ref(*m_session_ptr),
                                                              order_number);
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;

  ROS_INFO("Order Number: %s", order_number.getOrderNumber().c_str());
  return true;
}

bool SickSafetyscanners::requestProjectNameInColaSession(
  sick::datastructure::ProjectName& project_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::ProjectNameVariableCommand>(boost::ref(*m_session_ptr),
                                                              project_name);
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;

  ROS_INFO("Project Name: %s", project_name.getProjectName().c_str());
  return true;
}

bool SickSafetyscanners::requestUserNameInColaSession(sick::datastructure::UserName& user_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::UserNameVariableCommand>(boost::ref(*m_session_ptr), user_name);
  if (!m_session_ptr->executeCommand(command_ptr))
    return false;

  ROS_INFO("User Name: %s", user_name.getUserName().c_str());
  return true;
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

bool SickSafetyscanners::stopTCPConnection()
{
  if (!m_session_ptr->close())
    return false;

  return m_session_ptr->doDisconnect();
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
