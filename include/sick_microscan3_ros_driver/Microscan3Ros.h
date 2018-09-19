// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file Microscan3Ros.h
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-08-21
*
*/
#pragma once


// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>

//STD
#include <string>

//Package
#include <sick_microscan3_ros_driver/Microscan3.h>
#include <sick_microscan3_ros_driver/datastructure/CommSettings.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <dynamic_reconfigure/server.h>
#include <sick_microscan3_ros_driver/Microscan3ConfigurationConfig.h>

#include <sick_microscan3_ros_driver/RawMicroScanDataMsg.h>
#include <sick_microscan3_ros_driver/ExtendedLaserScanMsg.h>


#include <cmath>

namespace sick {

inline float degToRad(float deg) { return deg * M_PI / 180.0f; }
inline float radToDeg(float rad) { return rad * 180.0f / M_PI; }

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class Microscan3Ros
{
 public:
  /*!
   * Constructor.
   */
  Microscan3Ros();

  /*!
   * Destructor.
   */
  virtual ~Microscan3Ros();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void receivedUDPPaket(const datastructure::Data &data);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  void callback(sick_microscan3_ros_driver::Microscan3ConfigurationConfig &config, uint32_t level);

  bool isInitialised();

  bool m_initialised;
  //! ROS node handle.
  ros::NodeHandle m_nh;

  //! ROS private node handle
  ros::NodeHandle m_private_nh;

  //! ROS topic publisher
  ros::Publisher m_laser_scan_publisher;
  ros::Publisher m_extended_laser_scan_publisher;
  ros::Publisher m_raw_data_publisher;

  boost::shared_ptr<sick::Microscan3> m_device;

  sick::datastructure::CommSettings m_communication_settings;

  dynamic_reconfigure::Server<sick_microscan3_ros_driver::Microscan3ConfigurationConfig> m_dynamic_reconfiguration_server;

  std::string m_laser_scan_frame_name;

  sensor_msgs::LaserScan createLaserScanMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::ExtendedLaserScanMsg createExtendedLaserScanMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::RawMicroScanDataMsg createRawDataMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::DataHeaderMsg createDataHeaderMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::DerivedValuesMsg createDerivedValuesMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::GeneralSystemStateMsg createGeneralSystemStateMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::MeasurementDataMsg createMeasurementDataMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::ScanPointMsg createScanPointMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::IntrusionDataMsg createIntrusionDataMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::IntrusionDatumMsg createIntrusionDatumMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::ApplicationDataMsg createApplicationDataMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::ApplicationInputsMsg createApplicationInputsMessage(const sick::datastructure::Data &data);
  sick_microscan3_ros_driver::ApplicationOutputsMsg createApplicationOutputsMessage(const sick::datastructure::Data &data);
};

} /* namespace */
