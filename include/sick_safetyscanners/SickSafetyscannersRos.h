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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>

// STD
#include <string>
#include <vector>

// Package
#include <sick_safetyscanners/ExtendedLaserScanMsg.h>
#include <sick_safetyscanners/FieldData.h>
#include <sick_safetyscanners/OutputPathsMsg.h>
#include <sick_safetyscanners/RawMicroScanDataMsg.h>
#include <sick_safetyscanners/SickSafetyscanners.h>
#include <sick_safetyscanners/SickSafetyscannersConfigurationConfig.h>
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

  ros::ServiceServer m_field_service_server;

  bool m_initialised;

  std::shared_ptr<sick::SickSafetyscanners> m_device;

  sick::datastructure::CommSettings m_communication_settings;

  dynamic_reconfigure::Server<sick_safetyscanners::SickSafetyscannersConfigurationConfig>
    m_dynamic_reconfiguration_server;

  std::string m_laser_scan_frame_name;
  double m_range_min;
  double m_range_max;

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
  void callback(const sick_safetyscanners::SickSafetyscannersConfigurationConfig& config,
                const uint32_t& level);

  bool isInitialised();

  sensor_msgs::LaserScan createLaserScanMessage(const sick::datastructure::Data& data);
  sick_safetyscanners::ExtendedLaserScanMsg
  createExtendedLaserScanMessage(const sick::datastructure::Data& data);
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
  void readTypeCodeSettings();

  bool getFieldData(sick_safetyscanners::FieldData::Request& req,
                    sick_safetyscanners::FieldData::Response& res);
};

} // namespace sick

#endif // SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H
