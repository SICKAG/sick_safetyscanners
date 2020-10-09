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
 * \file ConfigData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-02-25
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGDATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGDATA_H

#include <boost/asio/ip/address_v4.hpp>
#include <iostream>
#include <sick_safetyscanners/datastructure/DerivedValues.h>
#include <vector>

namespace sick {
namespace datastructure {


/*!
 * \brief Config data for current and persistent sensor config.
 */
class ConfigData
{
public:
  /*!
   * \brief The constructor of the config data.
   */
  ConfigData();
  /*!
   * \brief Gets the version indicator for the scanner.
   *
   * \returns The version indicator for the scanner.
   */
  std::string getVersionCVersion() const;
  /*!
   * \brief Sets the version indicator for the scanner.
   *
   * \param version_c_version The version indicator for the scanner.
   */
  void setVersionCVersion(const std::string& version_c_version);

  /*!
   * \brief Gets the major version number for the scanner.
   *
   * \returns The version indicator for the scanner.
   */
  uint8_t getVersionMajorVersionNumber() const;
  /*!
   * \brief Sets the major version number for the scanner.
   *
   * \param version_major_version_number The major version number for the scanner.
   */
  void setVersionMajorVersionNumber(const uint8_t& version_major_version_number);

  /*!
   * \brief Gets the minor version number for the scanner.
   *
   * \returns The minor version number for the scanner.
   */
  uint8_t getVersionMinorVersionNumber() const;
  /*!
   * \brief Sets the minor version number for the scanner.
   *
   * \param version_minor_version_number The minor version number for the scanner.
   */
  void setVersionMinorVersionNumber(const uint8_t& version_minor_version_number);

  /*!
   * \brief Gets the version release number for the scanner.
   *
   * \returns The version release number for the scanner.
   */
  uint8_t getVersionReleaseNumber() const;
  /*!
   * \brief Sets the version release number for the scanner.
   *
   * \param version_release_number The version release number for the scanner.
   */
  void setVersionReleaseNumber(const uint8_t& version_release_number);

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
  float getStartAngle() const;
  /*!
   * \brief Sets the start angle of the scan.
   *
   * \param start_angle The start angle of the scan.
   */
  void setStartAngle(const int32_t& start_angle);

  /*!
   * \brief Set the start angle of the configuration from degrees.
   * \param start_angle Start angle of the configuration in degrees.
   */
  void setStartAngleDegrees(const float& start_angle);

  /*!
   * \brief Gets the end angle of the scan.
   *
   * \returns The end angle of the scan.
   */
  float getEndAngle() const;
  /*!
   * \brief Sets the end angle of the scan.
   *
   * \param end_angle The end angle of the scan.
   */
  void setEndAngle(const int32_t& end_angle);
  /*!
   * \brief Set the end angle of the configuration from degrees.
   * \param end_angle End angle of the configuration in degrees.
   */
  void setEndAngleDegrees(const float& end_angle);

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
   * \brief Return the multiplication factor.
   *  Multiplication factor to be applied to the beam distance values to
   *  get the distance in millimeter.
   * \return The multiplication factor.
   */
  uint16_t getDerivedMultiplicationFactor() const;

  /*!
   * \brief Sets the multiplication factor.
   * \param multiplication_factor The new multiplication factor.
   */
  void setDerivedMultiplicationFactor(const uint16_t& multiplication_factor);

  /*!
   * \brief Returns the number of beams of the current scan.
   * \return Number of beams.
   */
  uint16_t getDerivedNumberOfBeams() const;

  /*!
   * \brief Sets the number of beams for the current scan
   * \param number_of_beams Number of beams for the scan.
   */
  void setDerivedNumberOfBeams(const uint16_t& number_of_beams);

  /*!
   * \brief Return the time of the scan.
   * \return  Time of the scan.
   */
  uint16_t getDerivedScanTime() const;

  /*!
   * \brief Sets the time of the scan
   * \param scan_time Time of the scan.
   */
  void setDerivedScanTime(const uint16_t& scan_time);

  /*!
   * \brief Get the start angle of the scan.
   * \return Start angle of the scan.
   */
  float getDerivedStartAngle() const;

  /*!
   * \brief Set the start angle of the scan.
   * \param start_angle Start angle of the scan.
   */
  void setDerivedStartAngle(const int32_t& start_angle);

  /*!
   * \brief Returns the angular resolution between the beams.
   * \return Angular resolution between beams.
   */
  float getDerivedAngularBeamResolution() const;

  /*!
   * \brief Set the angular resolution between beams.
   * \param angular_beam_resolution The angular resolution between two beams.
   */
  void setDerivedAngularBeamResolution(const int32_t& angular_beam_resolution);

  /*!
   * \brief Set the angular resolution between beams from degrees.
   * \param angular_beam_resolution The angular resolution between two beams in degrees.
   */
  void setDerivedAngularBeamResolutionDegrees(const float& angular_beam_resolution);

  /*!
   * \brief Return the time between consecutive beams.
   * \return  Time between consecutive beams.
   */
  uint32_t getDerivedInterbeamPeriod() const;

  /*!
   * \brief Set the time between two consecutive beams.
   * \param interbeam_period Time between two consecutive beams.
   */
  void setDerivedInterbeamPeriod(const uint32_t& interbeam_period);

private:
  /*!
   * \brief Defined angle resolution to convert sensor input to the right frame
   */
  const double m_ANGLE_RESOLUTION = 4194304.0;

  // TODO cleanup and refactor in different subclasses

  std::string m_version_c_version;
  uint8_t m_version_major_version_number;
  uint8_t m_version_minor_version_number;
  uint8_t m_version_release_number;
  boost::asio::ip::address_v4 m_host_ip;
  uint16_t m_host_udp_port;
  uint8_t m_channel;
  bool m_enabled;
  uint8_t m_e_interface_type;
  uint16_t m_publishing_frequency;
  float m_start_angle;
  float m_end_angle;
  uint16_t m_features;
  uint16_t m_derived_multiplication_factor;
  uint16_t m_derived_number_of_beams;
  uint16_t m_derived_scan_time;
  float m_derived_start_angle;
  float m_derived_angular_beam_resolution; // TODO move into derived values
  uint16_t m_derived_interbeam_period;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGDATA_H
