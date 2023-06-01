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
 * \file SickSafetyscannersRos.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H
#define SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H


// ROS
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>

// STD
#include <string>
#include <vector>

// Package
#include <sick_safetyscanners/ConfigMetadata.h>
#include <sick_safetyscanners/ExtendedLaserScanMsg.h>
#include <sick_safetyscanners/FieldData.h>
#include <sick_safetyscanners/OutputPathsMsg.h>
#include <sick_safetyscanners/RawMicroScanDataMsg.h>
#include <sick_safetyscanners/SickSafetyscanners.h>
#include <sick_safetyscanners/SickSafetyscannersConfigurationConfig.h>
#include <sick_safetyscanners/StatusOverview.h>
#include <sick_safetyscanners/datastructure/CommSettings.h>
#include <sick_safetyscanners/datastructure/FieldData.h>

#include <dynamic_reconfigure/server.h>

#include <cmath>

namespace sick {

/*!
 * \brief Converts degrees to radians.
 * \param deg Degrees to convert.
 * \return To radians converted degrees.
 */
inline float degToRad(float deg)
{
  return deg * M_PI / 180.0f;
}

/*!
 * \brief Converts radians to degrees.
 * \param rad Input radians to convert
 * \return To degrees converted radians
 */
inline float radToDeg(float rad)
{
  return rad * 180.0f / M_PI;
}

/*!
 * \brief Converts a skip value into a "publish frequency" value
 * \param skip The number of scans to skip between each measured scan.  For a 25Hz laser, setting
 * 'skip' to 0 makes it publish at 25Hz, 'skip' to 1 makes it publish at 12.5Hz. \return "Publish
 * Frequency" ie. One out of every n_th scan will be published.  1 is publish every scan.  2 is
 * publish at half rate, and so on.
 */
inline uint16_t skipToPublishFrequency(int skip)
{
  return skip + 1;
}

typedef diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> DiagnosedLaserScanPublisher;

/*!
 * \brief The SickSafetyscannersRos class
 *
 * Main class for the node to handle the ROS interfacing.
 */
class SickSafetyscannersRos
{
public:
  /*!
   * \brief Constructor of the SickSafetyscannersRos
   *
   * Constructor of the SickSafetyscannersRos, loads all parameters from the parameter server,
   * initialises the dynamic reconfiguration server. Furthermore initialises the ROS Publishers for
   * the different laserscan outputs.
   */
  SickSafetyscannersRos();

  /*!
   * \brief ~SickSafetyscannersRos
   * Destructor if the SickSafetyscanners ROS
   */
  virtual ~SickSafetyscannersRos();

private:
  //! ROS node handle.
  ros::NodeHandle m_nh;

  //! ROS private node handle
  ros::NodeHandle m_private_nh;

  //! ROS topic publisher
  ros::Publisher m_laser_scan_publisher;
  ros::Publisher m_extended_laser_scan_publisher;
  ros::Publisher m_raw_data_publisher;
  ros::Publisher m_output_path_publisher;

  // ROS Timer
  ros::Timer m_udp_connection_monitor_timer;

  // Diagnostics
  diagnostic_updater::Updater m_diagnostic_updater;
  std::shared_ptr<DiagnosedLaserScanPublisher> m_diagnosed_laser_scan_publisher;
  sick_safetyscanners::RawMicroScanDataMsg m_last_raw_data;
  sick::datastructure::ConfigMetadata m_config_meta_data;
  sick::datastructure::FirmwareVersion m_firmware_version;
  void sensorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status);

  ros::ServiceServer m_field_service_server;
  ros::ServiceServer m_config_metadata_server;
  ros::ServiceServer m_status_overview_server;

  bool m_initialised;

  std::shared_ptr<sick::SickSafetyscanners> m_device;

  sick::datastructure::CommSettings m_communication_settings;
  boost::asio::ip::address_v4 m_interface_ip;

  dynamic_reconfigure::Server<sick_safetyscanners::SickSafetyscannersConfigurationConfig>
    m_dynamic_reconfiguration_server;

  std::string m_frame_id;
  double m_time_offset;
  double m_range_min;
  double m_range_max;
  double m_frequency_tolerance      = 0.1;
  double m_expected_frequency       = 20.0;
  double m_timestamp_min_acceptable = -1.0;
  double m_timestamp_max_acceptable = 1.0;
  double m_min_intensities          = 0.0; /*!< min intensities for laser points */

  bool m_use_sick_angles;
  float m_angle_offset;
  bool m_use_pers_conf;
  bool m_connection_status;
  
  uint32_t m_tcp_request_retry_ms;
  int m_tcp_max_request_retries;

  bool m_udp_connection_monitor;
  uint32_t m_connection_monitor_watchdog_timeout_ms;
  double m_last_udp_pkt_received;

  /*!
   * @brief Reads and verifies the ROS parameters.
   * @return True if successful.
   */
  bool readParameters();

  /*!
   * \brief Function which is called when a new complete UDP Packet is received
   * \param data, the assortment of all data from the sensor
   */
  void receivedUDPPacket(const datastructure::Data& data);

  /*!
   * \brief Function which is triggered when a dynamic reconfiguration is performed
   * \param config The new configuration to set
   * \param level Level of the new configuration
   */
  void reconfigureCallback(const sick_safetyscanners::SickSafetyscannersConfigurationConfig& config,
                           const uint32_t& level);

  bool isInitialised();

  sensor_msgs::LaserScan createLaserScanMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::ExtendedLaserScanMsg
  createExtendedLaserScanMessage(const sick::datastructure::Data& data);
  std::vector<bool>
  getMedianReflectors(const std::vector<sick::datastructure::ScanPoint> scan_points);
  sick_safetyscanners::OutputPathsMsg
  createOutputPathsMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::RawMicroScanDataMsg
  createRawDataMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::DataHeaderMsg createDataHeaderMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::DerivedValuesMsg
  createDerivedValuesMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::GeneralSystemStateMsg
  createGeneralSystemStateMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::MeasurementDataMsg
  createMeasurementDataMessage(const sick::datastructure::Data& data);
  std::vector<sick_safetyscanners::ScanPointMsg>
  createScanPointMessageVector(const sick::datastructure::Data& data);
  sick_safetyscanners::IntrusionDataMsg
  createIntrusionDataMessage(const sick::datastructure::Data& data);
  std::vector<sick_safetyscanners::IntrusionDatumMsg>
  createIntrusionDatumMessageVector(const sick::datastructure::Data& data);
  sick_safetyscanners::ApplicationDataMsg
  createApplicationDataMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::ApplicationInputsMsg
  createApplicationInputsMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::ApplicationOutputsMsg
  createApplicationOutputsMessage(const sick::datastructure::Data& data);
  bool readTypeCodeSettings();
  bool readPersistentConfig();

  bool getFieldData(sick_safetyscanners::FieldData::Request& req,
                    sick_safetyscanners::FieldData::Response& res);
  
  void udpConnectionMonitorHandler();
  void setCommunicationSettingScanner();

  bool getConfigMetadata(sick_safetyscanners::ConfigMetadata::Request& req,
                         sick_safetyscanners::ConfigMetadata::Response& res);

  bool getStatusOverview(sick_safetyscanners::StatusOverview::Request& req,
                         sick_safetyscanners::StatusOverview::Response& res);

  /**
   * @brief getCheckSumString converts the uint32 value received as checksum such that the
   * hexadecimal representation matches the value in the SafetyDesigner. To do this, we must invert
   * the order of the bytes
   * @param checksum as returned from API
   * @return hex string representing uint32 from reverted byte order
   */
  std::string getCheckSumString(uint32_t checksum);

  /**
   * @brief getDateString converts the date representation received as days since 1972-01-01 and
   * milliseconds since the start of that day to the form as given in the Cola2 documentation
   * @param days_since_1972 days from Jan 1, 1972 until day of transfer/modification
   * @param milli_seconds milliseconds since 0:00 on that day of transfer/modification
   * @return date string in the format YYYY-mm-DD HH:MM:SS
   */
  std::string getDateString(uint32_t days_since_1972, uint32_t milli_seconds);
};

} // namespace sick

#endif // SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H
