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
  packetReceivedCallbackFunction newPacketReceivedCallbackFunction,
  sick::datastructure::CommSettings settings)
  : m_newPacketReceivedCallbackFunction(newPacketReceivedCallbackFunction)
{
  ROS_INFO("Starting SickSafetyscanners");
  m_io_service_ptr       = std::make_shared<boost::asio::io_service>();
  m_async_udp_client_ptr = std::make_shared<sick::communication::AsyncUDPClient>(
    boost::bind(&SickSafetyscanners::processUDPPacket, this, _1),
    boost::ref(*m_io_service_ptr),
    settings.getHostUdpPort());
  m_packet_merger_ptr = std::make_shared<sick::data_processing::UDPPacketMerger>();
  ROS_INFO("Started SickSafetyscanners");
}

SickSafetyscanners::~SickSafetyscanners()
{
  m_udp_client_thread_ptr.reset();
}

bool SickSafetyscanners::run()
{
  m_udp_client_thread_ptr.reset(
    new boost::thread(boost::bind(&SickSafetyscanners::UDPClientThread, this)));

  m_async_udp_client_ptr->runService();
  return true;
}

bool SickSafetyscanners::UDPClientThread()
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

void SickSafetyscanners::requestTypeCode(const datastructure::CommSettings& settings,
                                         sick::datastructure::TypeCode& type_code)
{
  startTCPConnection(settings);

  requestTypeCodeInColaSession(type_code);

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
                                           std::string& device_name)
{
  startTCPConnection(settings);

  requestDeviceNameInColaSession(device_name);

  stopTCPConnection();
}

int SickSafetyscanners::getActiveCaseNumber() const
{
  return m_active_case_number;
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
  sick::datastructure::FieldData common_field_data;

  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::MeasurementCurrentConfigVariableCommand>(
      boost::ref(*m_session_ptr), common_field_data);
  m_session_ptr->executeCommand(command_ptr);

  command_ptr = std::make_shared<sick::cola2::MonitoringCaseTableHeaderVariableCommand>(
    boost::ref(*m_session_ptr), common_field_data);
  m_session_ptr->executeCommand(command_ptr);

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

      field_data.setStartAngleDegrees(common_field_data.getStartAngle());
      field_data.setAngularBeamResolutionDegrees(common_field_data.getAngularBeamResolution());

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

void SickSafetyscanners::requestDeviceNameInColaSession(std::string& device_name)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::DeviceNameVariableCommand>(boost::ref(*m_session_ptr),
                                                             device_name);
  m_session_ptr->executeCommand(command_ptr);
  ROS_INFO("Device name: %s", device_name.c_str());
}

void SickSafetyscanners::requestTypeCodeInColaSession(sick::datastructure::TypeCode& type_code)
{
  sick::cola2::Cola2Session::CommandPtr command_ptr =
    std::make_shared<sick::cola2::TypeCodeVariableCommand>(boost::ref(*m_session_ptr), type_code);
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
    sick::datastructure::PacketBuffer deployedBuffer =
      m_packet_merger_ptr->getDeployedPacketBuffer();
    sick::datastructure::Data data;
    sick::data_processing::ParseData data_parser;
    data_parser.parseUDPSequence(deployedBuffer, data);

    if (!data.getApplicationDataPtr()->isEmpty())
    {
      std::vector<uint16_t> monitoring_case_numbers =
        data.getApplicationDataPtr()->getOutputs().getMonitoringCaseVector();
      std::vector<bool> monitoring_case_number_flags =
        data.getApplicationDataPtr()->getOutputs().getMonitoringCaseFlagsVector();
      if (monitoring_case_number_flags.at(0))
      {
        m_active_case_number = monitoring_case_numbers.at(0);
      }
    }

    m_newPacketReceivedCallbackFunction(data);
  }
}

} // namespace sick
