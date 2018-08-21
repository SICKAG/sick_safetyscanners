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
#include <std_srvs/Trigger.h>

//STD
#include <string>

//Package
#include <sick_microscan3_ros_driver/Microscan3.h>

namespace sick {

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

  void receivedUDPPaket();

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle m_nh;

  //! ROS private node handle
  ros::NodeHandle m_private_nh;

  //! ROS topic subscriber.
  ros::Subscriber m_subscriber;

  //! ROS topic publisher
  ros::Publisher m_publisher;

  //! ROS topic and service names.
  std::string m_subscriber_topic;
  std::string m_publisher_topic;
  std::string m_service_server_name;
  std::string m_service_client_name;

  //! ROS service server.
  ros::ServiceServer m_service_server;

  //! ROS service client.
  ros::ServiceClient m_service_client;

};

} /* namespace */
