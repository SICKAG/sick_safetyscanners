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
* \file Microscan3Ros.cpp
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-09-24
*/
//----------------------------------------------------------------------

#include "sick_microscan3_ros_driver/Microscan3Ros.h"


namespace sick {

Microscan3Ros::Microscan3Ros()
    : m_nh()
    , m_private_nh("~")
    , m_initialised(false)
{
  dynamic_reconfigure::Server<sick_microscan3_ros_driver::Microscan3ConfigurationConfig>::CallbackType reconf_callback = boost::bind(&Microscan3Ros::callback, this,_1, _2);
  m_dynamic_reconfiguration_server.setCallback(reconf_callback);
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  m_device = boost::make_shared<sick::Microscan3>(boost::bind(&Microscan3Ros::receivedUDPPaket, this, _1), m_communication_settings);
  m_device->run();
  m_laser_scan_publisher = m_private_nh.advertise<sensor_msgs::LaserScan>("laser_scan",100);
  m_extended_laser_scan_publisher = m_private_nh.advertise<sick_microscan3_ros_driver::ExtendedLaserScanMsg>("extended_laser_scan", 100);
  m_device->changeSensorSettings(m_communication_settings);
  m_initialised = true;
  ROS_INFO("Successfully launched node.");
}

void Microscan3Ros::callback(sick_microscan3_ros_driver::Microscan3ConfigurationConfig &config, uint32_t level)
{
  if (isInitialised()) {
//    m_communication_settings.setHostIp(config.host_ip);
//    m_communication_settings.setHostUdpPort(config.host_udp_port);
    m_communication_settings.setChannel(config.channel);
    m_communication_settings.setEnabled(config.channel_enabled);
    m_communication_settings.setEInterfaceType(config.e_interface_type);
    m_communication_settings.setPublishingFequency(config.publish_frequency);
    m_communication_settings.setStartAngle(config.angle_start);
    m_communication_settings.setEndAngle(config.angle_end);
    m_communication_settings.setFeatures(config.general_system_state,
                                         config.derived_settings,
                                         config.measurement_data,
                                         config.intrusion_data,
                                         config.application_io_data);
    m_device->changeSensorSettings(m_communication_settings);

    m_laser_scan_frame_name = config.laser_scan_frame_name;
  }
}

bool Microscan3Ros::isInitialised()
{
  return m_initialised;
}


Microscan3Ros::~Microscan3Ros()
{
}

bool Microscan3Ros::readParameters()
{
  std::string sensor_ip_adress = sick_microscan3_ros_driver::Microscan3Configuration_sensor_ip;
  if (!m_private_nh.getParam("sensor_ip", sensor_ip_adress))
  {
//    sensor_ip_adress = sick_microscan3_ros_driver::Microscan3Configuration_sensor_ip;
    ROS_WARN("Using default sensor IP: %s", sensor_ip_adress.c_str());
  }
  m_communication_settings.setSensorIp(sensor_ip_adress);

  int sensor_tcp_port;
  if (!m_private_nh.getParam("sensor_tcp_port", sensor_tcp_port))
  {
    sensor_tcp_port = sick_microscan3_ros_driver::Microscan3Configuration_sensor_tcp_port;
    ROS_WARN("Using default sensor TCP port: %i", sensor_tcp_port);
  }
  m_communication_settings.setSensorTcpPort(sensor_tcp_port);


  std::string host_ip_adress = sick_microscan3_ros_driver::Microscan3Configuration_host_ip;
  if(!m_private_nh.getParam("host_ip", host_ip_adress))
  {
//    host_ip_adress = sick_microscan3_ros_driver::Microscan3Configuration_host_ip;
    ROS_WARN("Using default host IP: %s", host_ip_adress.c_str());
  }
  m_communication_settings.setHostIp(host_ip_adress);

  int host_udp_port;
  if(!m_private_nh.getParam("host_udp_port", host_udp_port))
  {
    host_udp_port = sick_microscan3_ros_driver::Microscan3Configuration_host_udp_port;
    ROS_WARN("Using default host UDP Port: %i", host_udp_port);
  }
  m_communication_settings.setHostUdpPort(host_udp_port);

  ROS_WARN("If not further specified the default values for the dynamic reconfigurable parameters will be loaded.");


  int channel;
  m_private_nh.getParam("channel", channel);
  m_communication_settings.setChannel(channel);

  bool enabled;
  m_private_nh.getParam("channel_enabled", enabled);
  m_communication_settings.setEnabled(enabled);

  int e_interface_type;
  m_private_nh.getParam("e_interface_type", e_interface_type);
  m_communication_settings.setEInterfaceType(e_interface_type);

  int publish_frequency;
  m_private_nh.getParam("publish_frequency", publish_frequency);
  m_communication_settings.setPublishingFequency(publish_frequency);

  float angle_start;
  m_private_nh.getParam("angle_start", angle_start);
  m_communication_settings.setStartAngle(angle_start);

  float angle_end;
  m_private_nh.getParam("angle_end", angle_end);
  m_communication_settings.setEndAngle(angle_end);

  bool general_system_state;
  m_private_nh.getParam("general_system_state", general_system_state);

  bool derived_settings;
  m_private_nh.getParam("derived_settings", derived_settings);

  bool measurement_data;
  m_private_nh.getParam("measurement_data", measurement_data);

  bool intrusion_data;
  m_private_nh.getParam("intrusion_data", intrusion_data);

  bool application_io_data;
  m_private_nh.getParam("application_io_data", application_io_data);

  m_communication_settings.setFeatures(general_system_state, derived_settings,measurement_data,intrusion_data,application_io_data);

  m_private_nh.getParam("laser_scan_frame_name",m_laser_scan_frame_name);

  return true;
}

void Microscan3Ros::receivedUDPPaket(const sick::datastructure::Data &data)
{
  //TODO send complex message, for each data packet one?
  if(!data.getMeasurementDataPtr()->isEmpty())
  {
    sensor_msgs::LaserScan scan = createLaserScanMessage(data);


    m_laser_scan_publisher.publish(scan);

//    sick_microscan3_ros_driver::ExtendedLaserScanMsg extendedScan = createExtendedLaserScanMessage(data);

//    m_extended_laser_scan_publisher.publish(extendedScan);
  }
}

sick_microscan3_ros_driver::ExtendedLaserScanMsg Microscan3Ros::createExtendedLaserScanMessage(const sick::datastructure::Data &data)
{
  sensor_msgs::LaserScan scan = createLaserScanMessage(data);
  sick_microscan3_ros_driver::ExtendedLaserScanMsg msg;
  msg.laser_scan = scan;

  int num_scan_points = data.getDerivedValuesPtr()->getNumberOfBeams();
  std::vector<sick::datastructure::ScanPoint> scan_points = data.getMeasurementDataPtr()->getScanPointsVector();

  for (int i = 0; i < num_scan_points; ++i)
  {
      const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
      msg.reflektor_status[i] = scan_point.getReflectorBit();
  }
}

sensor_msgs::LaserScan Microscan3Ros::createLaserScanMessage(const sick::datastructure::Data &data)
{
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = m_laser_scan_frame_name;
  scan.header.stamp = ros::Time::now();
  int num_scan_points = data.getDerivedValuesPtr()->getNumberOfBeams();

  scan.angle_min = sick::degToRad(data.getDerivedValuesPtr()->getStartAngle());
  scan.angle_max = sick::degToRad(data.getMeasurementDataPtr()->getScanPointsVector().at(data.getMeasurementDataPtr()->getScanPointsVector().size()-1).getAngle()); //TODO
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
      scan.ranges[i] = static_cast<float>(scan_point.getDistance()) * data.getDerivedValuesPtr()->getMultiplicationFactor() * 1e-3; // mm -> m
      scan.intensities[i] = static_cast<float>(scan_point.getReflectivity());
//              / std::numeric_limits<microscan3::ScanPointData::remission_type>::max();
  }

  return scan;
}

sick_microscan3_ros_driver::RawMicroScanDataMsg Microscan3Ros::createRawDataMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::DataHeaderMsg Microscan3Ros::createDataHeaderMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::DerivedValuesMsg Microscan3Ros::createDerivedValuesMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::GeneralSystemStateMsg Microscan3Ros::createGeneralSystemStateMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::MeasurementDataMsg Microscan3Ros::createMeasurementDataMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::ScanPointMsg Microscan3Ros::createScanPointMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::IntrusionDataMsg Microscan3Ros::createIntrusionDataMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::IntrusionDatumMsg Microscan3Ros::createIntrusionDatumMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::ApplicationDataMsg Microscan3Ros::createApplicationDataMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::ApplicationInputsMsg Microscan3Ros::createApplicationInputsMessage(const sick::datastructure::Data &data)
{

}

sick_microscan3_ros_driver::ApplicationOutputsMsg Microscan3Ros::createApplicationOutputsMessage(const sick::datastructure::Data &data)
{

}



} /* namespace */
