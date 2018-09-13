#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <iostream>

namespace sick {
namespace datastructure {

class CommSettings
{
public:
  CommSettings();

  boost::asio::ip::address_v4 getHostIp() const;
  void setHostIp(const boost::asio::ip::address_v4 &host_ip);
  void setHostIp(const std::string &host_ip);

  UINT16 getHostUdpPort() const;
  void setHostUdpPort(const UINT16 &host_udp_port);

  UINT8 getChannel() const;
  void setChannel(const UINT8 &channel);

  bool getEnabled() const;
  void setEnabled(bool enabled);

  UINT8 getEInterfaceType() const;
  void setEInterfaceType(const UINT8 &e_interface_type);

  UINT16 getPublishingFequency() const;
  void setPublishingFequency(const UINT16 &publishing_fequency);

  UINT32 getStartAngle() const;
  void setStartAngle(const UINT32 &start_angle);

  UINT32 getEndAngle() const;
  void setEndAngle(const UINT32 &end_angle);

  UINT16 getFeatures() const;
  void setFeatures(const UINT16 &features);
  void setFeatures(const bool general_system_state, const bool derived_settings, const bool measurement_data,const bool intrusion_data,const bool application_data);

  boost::asio::ip::address_v4 getSensorIp() const;
  void setSensorIp(const boost::asio::ip::address_v4 &sensor_ip);

  UINT16 getSensorTcpPort() const;
  void setSensorTcpPort(const UINT16 &sensor_tcp_port);
  void setSensorIp(const std::__cxx11::string &host_ip);

private:
  boost::asio::ip::address_v4 m_sensor_ip;
  UINT16 m_sensor_tcp_port;
  boost::asio::ip::address_v4 m_host_ip;
  UINT16 m_host_udp_port;
  UINT8 m_channel;
  bool m_enabled;
  UINT8 m_e_interface_type;
  UINT16 m_publishing_fequency;
  UINT32 m_start_angle;
  UINT32 m_end_angle;
  UINT16 m_features;

};


}
}
