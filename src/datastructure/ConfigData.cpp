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
 * \file ConfigData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-02-25
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/ConfigData.h>

namespace sick {
namespace datastructure {

ConfigData::ConfigData() {}

std::string ConfigData::getVersionCVersion() const
{
  return m_version_c_version;
}

void ConfigData::setVersionCVersion(const std::string& version_c_version)
{
  m_version_c_version = version_c_version;
}

uint8_t ConfigData::getVersionMajorVersionNumber() const
{
  return m_version_major_version_number;
}

void ConfigData::setVersionMajorVersionNumber(const uint8_t& version_major_version_number)
{
  m_version_major_version_number = version_major_version_number;
}

uint8_t ConfigData::getVersionMinorVersionNumber() const
{
  return m_version_minor_version_number;
}

void ConfigData::setVersionMinorVersionNumber(const uint8_t& version_minor_version_number)
{
  m_version_minor_version_number = version_minor_version_number;
}

uint8_t ConfigData::getVersionReleaseNumber() const
{
  return m_version_release_number;
}

void ConfigData::setVersionReleaseNumber(const uint8_t& version_release_number)
{
  m_version_release_number = version_release_number;
}

boost::asio::ip::address_v4 ConfigData::getHostIp() const
{
  return m_host_ip;
}

void ConfigData::setHostIp(const boost::asio::ip::address_v4& host_ip)
{
  m_host_ip = host_ip;
}

void ConfigData::setHostIp(const std::string& host_ip)
{
  m_host_ip = boost::asio::ip::address_v4::from_string(host_ip);
}

uint16_t ConfigData::getHostUdpPort() const
{
  return m_host_udp_port;
}

void ConfigData::setHostUdpPort(const uint16_t& host_udp_port)
{
  m_host_udp_port = host_udp_port;
}

uint8_t ConfigData::getChannel() const
{
  return m_channel;
}

void ConfigData::setChannel(const uint8_t& channel)
{
  m_channel = channel;
}

bool ConfigData::getEnabled() const
{
  return m_enabled;
}

void ConfigData::setEnabled(bool enabled)
{
  m_enabled = enabled;
}

uint8_t ConfigData::getEInterfaceType() const
{
  return m_e_interface_type;
}

void ConfigData::setEInterfaceType(const uint8_t& e_interface_type)
{
  m_e_interface_type = e_interface_type;
}

uint16_t ConfigData::getPublishingFrequency() const
{
  return m_publishing_frequency;
}

void ConfigData::setPublishingFrequency(const uint16_t& publishing_frequency)
{
  m_publishing_frequency = publishing_frequency;
}

uint16_t ConfigData::getFeatures() const
{
  return m_features;
}

void ConfigData::setFeatures(const uint16_t& features)
{
  m_features = features;
}

void ConfigData::setFeatures(bool general_system_state,
                             bool derived_settings,
                             bool measurement_data,
                             bool intrusion_data,
                             bool application_data)
{
  m_features = 0;
  m_features += (static_cast<int>(general_system_state) << 0);
  m_features += (static_cast<int>(derived_settings) << 1);
  m_features += (static_cast<int>(measurement_data) << 2);
  m_features += (static_cast<int>(intrusion_data) << 3);
  m_features += (static_cast<int>(application_data) << 4);
}

float ConfigData::getStartAngle() const
{
  return m_start_angle;
}

void ConfigData::setStartAngle(const int32_t& start_angle)
{
  m_start_angle = (float)start_angle / m_ANGLE_RESOLUTION;
}

void ConfigData::setStartAngleDegrees(const float& start_angle)
{
  m_start_angle = start_angle;
}

float ConfigData::getEndAngle() const
{
  return m_end_angle;
}

void ConfigData::setEndAngle(const int32_t& end_angle)
{
  m_end_angle = (float)end_angle / m_ANGLE_RESOLUTION;
}

void ConfigData::setEndAngleDegrees(const float& end_angle)
{
  m_end_angle = end_angle;
}

/*float ConfigData::getAngularBeamResolution() const
{
  return m_angular_beam_resolution;
}

void ConfigData::setAngularBeamResolution(const int32_t& angular_beam_resolution)
{
  m_angular_beam_resolution = (float)angular_beam_resolution / m_ANGLE_RESOLUTION;
}

void ConfigData::setAngularBeamResolutionDegrees(const float& angular_beam_resolution)
{
  m_angular_beam_resolution = angular_beam_resolution;
}*/


uint16_t ConfigData::getDerivedMultiplicationFactor() const
{
  return m_derived_multiplication_factor;
}

void ConfigData::setDerivedMultiplicationFactor(const uint16_t& multiplication_factor)
{
  m_derived_multiplication_factor = multiplication_factor;
}

uint16_t ConfigData::getDerivedNumberOfBeams() const
{
  return m_derived_number_of_beams;
}

void ConfigData::setDerivedNumberOfBeams(const uint16_t& number_of_beams)
{
  m_derived_number_of_beams = number_of_beams;
}

uint16_t ConfigData::getDerivedScanTime() const
{
  return m_derived_scan_time;
}

void ConfigData::setDerivedScanTime(const uint16_t& scan_time)
{
  m_derived_scan_time = scan_time;
}


float ConfigData::getDerivedStartAngle() const
{
  return m_derived_start_angle;
}

void ConfigData::setDerivedStartAngle(const int32_t& start_angle)
{
  m_derived_start_angle = (float)start_angle / m_ANGLE_RESOLUTION;
}

float ConfigData::getDerivedAngularBeamResolution() const
{
  return m_derived_angular_beam_resolution;
}

void ConfigData::setDerivedAngularBeamResolution(const int32_t& angular_beam_resolution)
{
  m_derived_angular_beam_resolution = (float)angular_beam_resolution / m_ANGLE_RESOLUTION;
}

void ConfigData::setDerivedAngularBeamResolutionDegrees(const float& angular_beam_resolution)
{
  m_derived_angular_beam_resolution = angular_beam_resolution;
}

uint32_t ConfigData::getDerivedInterbeamPeriod() const
{
  return m_derived_interbeam_period;
}

void ConfigData::setDerivedInterbeamPeriod(const uint32_t& interbeam_period)
{
  m_derived_interbeam_period = interbeam_period;
}

} // namespace datastructure
} // namespace sick
