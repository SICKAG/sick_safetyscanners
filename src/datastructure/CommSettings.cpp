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
* \file CommSettings.cpp
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-09-24
*/
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/datastructure/CommSettings.h>

namespace sick {
namespace datastructure {

CommSettings::CommSettings()
{

}

boost::asio::ip::address_v4 CommSettings::getHostIp() const
{
  return m_host_ip;
}

void CommSettings::setHostIp(const boost::asio::ip::address_v4 &host_ip)
{
  m_host_ip = host_ip;
}

void CommSettings::setHostIp(const std::__cxx11::string &host_ip)
{
  m_host_ip = boost::asio::ip::address_v4::from_string(host_ip);
}

UINT16 CommSettings::getHostUdpPort() const
{
  return m_host_udp_port;
}

void CommSettings::setHostUdpPort(const UINT16 &host_udp_port)
{
  m_host_udp_port = host_udp_port;
}

UINT8 CommSettings::getChannel() const
{
  return m_channel;
}

void CommSettings::setChannel(const UINT8 &channel)
{
  m_channel = channel;
}

bool CommSettings::getEnabled() const
{
  return m_enabled;
}

void CommSettings::setEnabled(bool enabled)
{
  m_enabled = enabled;
}

UINT8 CommSettings::getEInterfaceType() const
{
  return m_e_interface_type;
}

void CommSettings::setEInterfaceType(const UINT8 &e_interface_type)
{
  m_e_interface_type = e_interface_type;
}

UINT16 CommSettings::getPublishingFequency() const
{
  return m_publishing_fequency;
}

void CommSettings::setPublishingFequency(const UINT16 &publishing_fequency)
{
  m_publishing_fequency = publishing_fequency;
}

UINT32 CommSettings::getStartAngle() const
{
  return m_start_angle;
}

void CommSettings::setStartAngle(const UINT32 &start_angle)
{
  m_start_angle = start_angle * 4194304.0;
}

UINT32 CommSettings::getEndAngle() const
{
  return m_end_angle;
}

void CommSettings::setEndAngle(const UINT32 &end_angle)
{
  m_end_angle = end_angle * 4194304.0;
}

UINT16 CommSettings::getFeatures() const
{
  return m_features;
}

void CommSettings::setFeatures(const UINT16 &features)
{
  m_features = features;
}

void CommSettings::setFeatures(const bool general_system_state, const bool derived_settings, const bool measurement_data, const bool intrusion_data, const bool application_data)
{
  m_features = 0;
  m_features += general_system_state << 0;
  m_features += derived_settings << 1;
  m_features += measurement_data << 2;
  m_features += intrusion_data << 3;
  m_features += application_data << 4;
}

boost::asio::ip::address_v4 CommSettings::getSensorIp() const
{
  return m_sensor_ip;
}

void CommSettings::setSensorIp(const boost::asio::ip::address_v4 &sensor_ip)
{
  m_sensor_ip = sensor_ip;
}

void CommSettings::setSensorIp(const std::__cxx11::string &host_ip)
{
  m_sensor_ip = boost::asio::ip::address_v4::from_string(host_ip);
}

UINT16 CommSettings::getSensorTcpPort() const
{
  return m_sensor_tcp_port;
}

void CommSettings::setSensorTcpPort(const UINT16 &sensor_tcp_port)
{
  m_sensor_tcp_port = sensor_tcp_port;
}


}
}

