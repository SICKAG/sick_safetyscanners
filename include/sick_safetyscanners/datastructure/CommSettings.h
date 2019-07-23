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

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_COMMSETTINGS_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_COMMSETTINGS_H

#include <boost/asio/ip/address_v4.hpp>
#include <iostream>
#include <string>

namespace sick {
namespace datastructure {

/*!
 * \brief Containing the communication settings for the sensor which can be changed on runtime.
 */
class CommSettings
{
public:
  /*!
   * \brief Constructor of the communication settings.
   */
  CommSettings();

  /*!
   * \brief Gets the IP-address of the host.
   *
   * \returns The IP-address of the host.
   */
  boost::asio::ip::address_v4 getHostIp() const;
  /*!
   * \brief Sets the IP-address of the host from an IP-address.
   *
   * \param host_ip The new host IP-address.
   */
  void setHostIp(const boost::asio::ip::address_v4& host_ip);
  /*!
   * \brief Sets the IP-address of the host from a string.
   *
   * \param host_ip The new host IP-address.
   */
  void setHostIp(const std::string& host_ip);

  /*!
   * \brief Gets the host udp port.
   *
   * \returns The host udp port.
   */
  uint16_t getHostUdpPort() const;
  /*!
   * \brief Sets the host udp port.
   *
   * \param host_udp_port The new host udp port.
   */
  void setHostUdpPort(const uint16_t& host_udp_port);

  /*!
   * \brief Gets the channel of the data.
   *
   * \returns The channel of the data.
   */
  uint8_t getChannel() const;
  /*!
   * \brief Sets the channel of the data.
   *
   * \param channel The new channel.
   */
  void setChannel(const uint8_t& channel);

  /*!
   * \brief Gets if the channel is enabled.
   *
   * \returns If the channel is enabled.
   */
  bool getEnabled() const;
  /*!
   * \brief Sets if the channel is enabled.
   *
   * \param enabled If the channel is enabled.
   */
  void setEnabled(bool enabled);

  /*!
   * \brief Gets the eInterface type.
   *
   * \returns The eInterface type.
   */
  uint8_t getEInterfaceType() const;
  /*!
   * \brief Sets the eInterface type.
   *
   * \param e_interface_type The new eInterface type.
   */
  void setEInterfaceType(const uint8_t& e_interface_type);

  /*!
   * \brief Gets the publishing frequency.
   *
   * \returns The publishing frequency.
   */
  uint16_t getPublishingFrequency() const;
  /*!
   * \brief Sets the publishing frequency.
   *
   * \param publishing_frequency The publishing frequency.
   */
  void setPublishingFrequency(const uint16_t& publishing_frequency);

  /*!
   * \brief Gets the start angle of the scan.
   *
   * \returns The start angle of the scan.
   */
  uint32_t getStartAngle() const;
  /*!
   * \brief Sets the start angle of the scan.
   *
   * \param start_angle The start angle of the scan.
   */
  void setStartAngle(const uint32_t& start_angle);

  /*!
   * \brief Gets the end angle of the scan.
   *
   * \returns The end angle of the scan.
   */
  uint32_t getEndAngle() const;
  /*!
   * \brief Sets the end angle of the scan.
   *
   * \param end_angle The end angle of the scan.
   */
  void setEndAngle(const uint32_t& end_angle);

  /*!
   * \brief Gets the enabled features.
   *
   * \returns The enabled features.
   */
  uint16_t getFeatures() const;
  /*!
   * \brief Set the enabled features.
   *
   * \param features The new enabled features.
   */
  void setFeatures(const uint16_t& features);
  /*!
   * \brief Sets the enabled features.
   *
   * \param general_system_state If general system state is enabled.
   * \param derived_settings If derived settings are enabled.
   * \param measurement_data If the measurement data is enabled.
   * \param intrusion_data If intrusion data is enabled.
   * \param application_data If application data is enabled.
   */
  void setFeatures(bool general_system_state,
                   bool derived_settings,
                   bool measurement_data,
                   bool intrusion_data,
                   bool application_data);

  /*!
   * \brief Gets the sensor IP-address.
   *
   * \returns The sensor IP-address.
   */
  boost::asio::ip::address_v4 getSensorIp() const;
  /*!
   * \brief Sets the sensor IP-address.
   *
   * \param sensor_ip The sensor IP-address.
   */
  void setSensorIp(const boost::asio::ip::address_v4& sensor_ip);

  /*!
   * \brief Gets the sensor tcp port.
   *
   * \returns The sensor tcp port.
   */
  uint16_t getSensorTcpPort() const;
  /*!
   * \brief Sets the sensor tcp port.
   *
   * \param sensor_tcp_portThe sensor tcp port.
   */
  void setSensorTcpPort(const uint16_t& sensor_tcp_port);
  /*!
   * \brief Sets the sensor IP-address from a string.
   *
   * \param sensor_ip Sets the sensor IP-address.
   */
  void setSensorIp(const std::string& sensor_ip);

private:
  boost::asio::ip::address_v4 m_sensor_ip;
  uint16_t m_sensor_tcp_port;
  boost::asio::ip::address_v4 m_host_ip;
  uint16_t m_host_udp_port;
  uint8_t m_channel;
  bool m_enabled;
  uint8_t m_e_interface_type;
  uint16_t m_publishing_frequency;
  uint32_t m_start_angle;
  uint32_t m_end_angle;
  uint16_t m_features;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_COMMSETTINGS_H
