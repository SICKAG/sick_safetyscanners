// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file Microscan3Ros.cpp
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-08-21
*
*/

#include "sick_microscan3_ros_driver/Microscan3Ros.h"

namespace sick {

Microscan3Ros::Microscan3Ros()
    : m_nh()
    , m_private_nh("~")
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }


   m_device = boost::make_shared<sick::Microscan3>(std::bind(&Microscan3Ros::receivedUDPPaket, this));
   m_device->run();
/*  m_subscriber = m_nh.subscribe(m_subscriber_topic, 1,
                                      &RosPackageTemplate::topicCallback, this);
  m_publisher = m_private_nh.advertise<sensor_msgs::JointState>(m_publisher_topic,100);
*/
  m_service_server = m_private_nh.advertiseService("m_service_server_name",
                                                &Microscan3Ros::serviceCallback, this);
/*
  //TODO uncomment for real test
  //ros::service::waitForService(m_service_client_name);
  m_service_client = m_nh.serviceClient<std_srvs::Trigger>(m_service_client_name);
  */
  ROS_INFO("Successfully launched node.");
}

Microscan3Ros::~Microscan3Ros()
{
}

bool Microscan3Ros::readParameters()
{
  /*
  if (!m_private_nh.getParam("topics/incoming/joint_states", m_subscriber_topic)) return false;
  if (!m_private_nh.getParam("topics/outgoing/joint_states", m_publisher_topic)) return false;
  if (!m_private_nh.getParam("services/server/calculate", m_service_server_name)) return false;
  if (!m_private_nh.getParam("services/client/get_data", m_service_client_name)) return false;
  */
  return true;
}

void Microscan3Ros::receivedUDPPaket()
{
  ROS_INFO("Received UDP Paket");
}

bool Microscan3Ros::serviceCallback(std_srvs::Trigger::Request& request,
      std_srvs::Trigger::Response& response)
{
  ROS_INFO("Received Service Call");
  response.success = true;
  return true;
}

} /* namespace */
