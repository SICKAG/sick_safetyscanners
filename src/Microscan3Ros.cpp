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

//TODO publish PointCLoud2 as well?

Microscan3Ros::Microscan3Ros()
    : m_nh()
    , m_private_nh("~")
    , m_initialised(false)
{
  dynamic_reconfigure::Server<sick_microscan3_ros_driver::Microscan3ConfigurationConfig>::CallbackType f = boost::bind(&Microscan3Ros::callback, this,_1, _2);
  srvs.setCallback(f);

  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }


   m_device = boost::make_shared<sick::Microscan3>(boost::bind(&Microscan3Ros::receivedUDPPaket, this, _1), m_communication_settings);
   m_device->run();



  m_publisher = m_private_nh.advertise<sensor_msgs::LaserScan>("laser_scan",100);

  m_service_server = m_private_nh.advertiseService("m_service_server_name",
                                                &Microscan3Ros::serviceCallback, this);


  m_device->serviceTCP(m_communication_settings);

  m_initialised = true;

  ROS_INFO("Successfully launched node.");
}

void Microscan3Ros::callback(sick_microscan3_ros_driver::Microscan3ConfigurationConfig &config, uint32_t level)
{
  if (isInitialised()) {
    m_communication_settings.setHostIp(config.host_ip);
    m_communication_settings.setHostUdpPort(config.host_udp_port);
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

    m_device->serviceTCP(m_communication_settings);
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
    sensor_ip_adress = sick_microscan3_ros_driver::Microscan3Configuration_sensor_ip;
    ROS_WARN("Using default sensor IP: %s", sensor_ip_adress.c_str());
  }
  m_communication_settings.setSensorIp(sensor_ip_adress);

  std::string sensor_tcp_port;
  if (!m_private_nh.getParam("sensor_tcp_port", sensor_tcp_port))
  {
    sensor_tcp_port = sick_microscan3_ros_driver::Microscan3Configuration_sensor_tcp_port;
    ROS_WARN("Using default sensor TCP port: %s", sensor_tcp_port.c_str());
  }
  m_communication_settings.setSensorTcpPort(std::stoi(sensor_tcp_port));

  std::string host_ip_adress;
  if (!m_private_nh.getParam("host_ip", host_ip_adress))
  {
    ROS_WARN("Using default host IP: %s", host_ip_adress.c_str());
  }
  m_communication_settings.setHostIp(host_ip_adress);

  int host_udp_port;
  if (!m_private_nh.getParam("host_udp_port", host_udp_port))
  {
    ROS_WARN("Using default host UDP port: %i", host_udp_port);
  }
  m_communication_settings.setHostUdpPort(host_udp_port);

  int channel;
  if (!m_private_nh.getParam("channel", channel))
  {
    ROS_WARN("Using default channel: %i", channel);
  }
  m_communication_settings.setChannel(channel);

  bool enabled;
  if (!m_private_nh.getParam("channel_enabled", enabled))
  {
    ROS_WARN("Channel enabled by default");
  }
  m_communication_settings.setEnabled(enabled);

  int e_interface_type;
  if (!m_private_nh.getParam("e_interface_type", e_interface_type))
  {
    ROS_WARN("Using default eInterfaceType %i", e_interface_type);
  }
  m_communication_settings.setEInterfaceType(e_interface_type);

  int publish_frequency;
  if (!m_private_nh.getParam("publish_frequency", publish_frequency))
  {
    ROS_WARN("Using default publish frequency: %i", publish_frequency);
  }
  m_communication_settings.setPublishingFequency(publish_frequency);

  float angle_start;
  if (!m_private_nh.getParam("angle_start", angle_start))
  {
    ROS_WARN("Using default start angle: %f", angle_start);
  }
  m_communication_settings.setStartAngle(angle_start);

  float angle_end;
  if (!m_private_nh.getParam("angle_end", angle_end))
  {
    ROS_WARN("Using default end angle: %f", angle_end);
  }
  m_communication_settings.setEndAngle(angle_end);

  bool general_system_state;
  if (!m_private_nh.getParam("general_system_state", general_system_state))
  {
    ROS_WARN("General System State enabled by default");
  }

  bool derived_settings;
  if (!m_private_nh.getParam("derived_settings", derived_settings))
  {
    ROS_WARN("Derived Settings enabled by default");
  }

  bool measurement_data;
  if (!m_private_nh.getParam("measurement_data", measurement_data))
  {
    ROS_WARN("Measurement Data enabled by default");
  }

  bool intrusion_data;
  if (!m_private_nh.getParam("intrusion_data", intrusion_data))
  {
    ROS_WARN("Intrusion Data enabled by default");
  }

  bool application_io_data;
  if (!m_private_nh.getParam("application_io_data", application_io_data))
  {
    ROS_WARN("Application IO Data enabled by default");
  }

  m_communication_settings.setFeatures(general_system_state, derived_settings,measurement_data,intrusion_data,application_io_data);

  return true;
}

void Microscan3Ros::receivedUDPPaket(const sick::datastructure::Data &data)
{
  //TODO
  ROS_INFO("Received UDP Paket");
  if(!data.getMeasurementDataPtr()->isEmpty())
  {

    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "laser_scan"; //TODO
    scan.header.stamp = ros::Time::now();
    int num_scan_points = data.getDerivedValuesPtr()->getNumberOfBeams();

    scan.angle_min = sick::degToRad(data.getMeasurementDataPtr()->getScanPointsVector().at(0).getAngle());
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
}

bool Microscan3Ros::serviceCallback(std_srvs::Trigger::Request& request,
      std_srvs::Trigger::Response& response)
{  
  ROS_INFO("Received Service Call");

  m_device->serviceTCP(m_communication_settings);
  response.success = true;
  return true;
}

} /* namespace */
