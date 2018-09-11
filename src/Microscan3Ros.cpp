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


   m_device = boost::make_shared<sick::Microscan3>(boost::bind(&Microscan3Ros::receivedUDPPaket, this, _1));
   m_device->run();
/*  m_subscriber = m_nh.subscribe(m_subscriber_topic, 1,
                                      &RosPackageTemplate::topicCallback, this);

                                      */
  m_publisher = m_private_nh.advertise<sensor_msgs::LaserScan>("laser_scan",100);

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

void Microscan3Ros::receivedUDPPaket(const sick::datastructure::Data &data)
{
  ROS_INFO("Received UDP Paket");

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser_scan"; //TODO
  scan.header.stamp = ros::Time::now();
  int num_scan_points = data.getDerivedValuesPtr()->getNumberOfBeams();

  scan.angle_min = sick::degToRad(data.getMeasurementDataPtr()->getScanPointsVector().at(0).getAngle());
  std::cout << "data ros: " << data.getDerivedValuesPtr()->getAngularBeamResolution();
  scan.angle_max = sick::degToRad(data.getMeasurementDataPtr()->getScanPointsVector().at(data.getMeasurementDataPtr()->getScanPointsVector().size()-1).getAngle());
  scan.angle_increment = sick::degToRad(data.getDerivedValuesPtr()->getAngularBeamResolution());
  boost::posix_time::microseconds time_increment = boost::posix_time::microseconds(data.getDerivedValuesPtr()->getInterbeamPeriod());
  scan.time_increment = time_increment.total_microseconds() * 1e-6;
  boost::posix_time::milliseconds scan_time = boost::posix_time::milliseconds(data.getDerivedValuesPtr()->getScanTime());
  scan.scan_time = scan_time.total_microseconds() * 1e-6;
  scan.range_min = 0.02; // TODO configurable, values taken from reichweite und benötigte remissionen für Warnfelder from SICK datasheet
  scan.range_max = 40.0; // TODO
  scan.ranges.resize(num_scan_points);
  scan.intensities.resize(num_scan_points);


  std::vector<sick::datastructure::ScanPoint> scan_points = data.getMeasurementDataPtr()->getScanPointsVector();
  for (int i = 0; i < num_scan_points; ++i)
  {
      const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
      scan.ranges[i] = static_cast<float>(scan_point.getDistance()) * 1e-3; // mm -> m
      scan.intensities[i] = static_cast<float>(scan_point.getReflectivity());
//              / std::numeric_limits<microscan3::ScanPointData::remission_type>::max();
  }
  m_publisher.publish(scan);
}

bool Microscan3Ros::serviceCallback(std_srvs::Trigger::Request& request,
      std_srvs::Trigger::Response& response)
{
  ROS_INFO("Received Service Call");

  m_device->serviceTCP();
  response.success = true;
  return true;
}

} /* namespace */
