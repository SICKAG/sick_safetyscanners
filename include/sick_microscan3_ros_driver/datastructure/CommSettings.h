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

#ifndef COMMSETTINGS_H
#define COMMSETTINGS_H

#include <boost/asio/ip/address_v4.hpp>
#include <iostream>

namespace sick {
namespace datastructure {

class CommSettings
{
public:
  CommSettings();

  boost::asio::ip::address_v4 getHostIp() const;
  void setHostIp(const boost::asio::ip::address_v4& host_ip);
  void setHostIp(const std::string& host_ip);

  uint16_t getHostUdpPort() const;
  void setHostUdpPort(const uint16_t& host_udp_port);

  uint8_t getChannel() const;
  void setChannel(const uint8_t& channel);

  bool getEnabled() const;
  void setEnabled(bool enabled);

  uint8_t getEInterfaceType() const;
  void setEInterfaceType(const uint8_t& e_interface_type);

  uint16_t getPublishingFequency() const;
  void setPublishingFequency(const uint16_t& publishing_fequency);

  uint32_t getStartAngle() const;
  void setStartAngle(const uint32_t& start_angle);

  uint32_t getEndAngle() const;
  void setEndAngle(const uint32_t& end_angle);

  uint16_t getFeatures() const;
  void setFeatures(const uint16_t& features);
  void setFeatures(const bool general_system_state,
                   const bool derived_settings,
                   const bool measurement_data,
                   const bool intrusion_data,
                   const bool application_data);

  boost::asio::ip::address_v4 getSensorIp() const;
  void setSensorIp(const boost::asio::ip::address_v4& sensor_ip);

  uint16_t getSensorTcpPort() const;
  void setSensorTcpPort(const uint16_t& sensor_tcp_port);
  void setSensorIp(const std::string& host_ip);

private:
  boost::asio::ip::address_v4 m_sensor_ip;
  uint16_t m_sensor_tcp_port;
  boost::asio::ip::address_v4 m_host_ip;
  uint16_t m_host_udp_port;
  uint8_t m_channel;
  bool m_enabled;
  uint8_t m_e_interface_type;
  uint16_t m_publishing_fequency;
  uint32_t m_start_angle;
  uint32_t m_end_angle;
  uint16_t m_features;
};


} // namespace datastructure
} // namespace sick

#endif
