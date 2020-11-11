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
 * \file SickSafetyscannersRos.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#include "sick_safetyscanners/SickSafetyscannersRos.h"


namespace sick {

SickSafetyscannersRos::SickSafetyscannersRos()
  : m_private_nh("~")
  , m_initialised(false)
  , m_time_offset(0.0)
  , m_range_min(0.0)
  , m_range_max(0.0)
  , m_angle_offset(-90.0)
  , m_use_pers_conf(false)
{
  dynamic_reconfigure::Server<
    sick_safetyscanners::SickSafetyscannersConfigurationConfig>::CallbackType reconf_callback =
    boost::bind(&SickSafetyscannersRos::reconfigureCallback, this, _1, _2);
  m_dynamic_reconfiguration_server.setCallback(reconf_callback);
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  // tcp port can not be changed in the sensor configuration, therefore it is hardcoded
  m_communication_settings.setSensorTcpPort(2122);
  m_laser_scan_publisher = m_nh.advertise<sensor_msgs::LaserScan>("scan", 100);
  m_extended_laser_scan_publisher =
    m_nh.advertise<sick_safetyscanners::ExtendedLaserScanMsg>("extended_laser_scan", 100);
  m_raw_data_publisher = m_nh.advertise<sick_safetyscanners::RawMicroScanDataMsg>("raw_data", 100);
  m_output_path_publisher =
    m_nh.advertise<sick_safetyscanners::OutputPathsMsg>("output_paths", 100);
  m_field_service_server =
    m_nh.advertiseService("field_data", &SickSafetyscannersRos::getFieldData, this);

  // Diagnostics for frequency
  m_diagnostic_updater.setHardwareID(m_communication_settings.getSensorIp().to_string());

  diagnostic_updater::FrequencyStatusParam frequency_param(
    &m_expected_frequency, &m_expected_frequency, m_frequency_tolerance);
  diagnostic_updater::TimeStampStatusParam timestamp_param(m_timestamp_min_acceptable,
                                                           m_timestamp_max_acceptable);
  m_diagnosed_laser_scan_publisher.reset(new DiagnosedLaserScanPublisher(
    m_laser_scan_publisher, m_diagnostic_updater, frequency_param, timestamp_param));
  m_diagnostic_updater.add("State", this, &SickSafetyscannersRos::sensorDiagnostics);

  m_device = std::make_shared<sick::SickSafetyscanners>(
    boost::bind(&SickSafetyscannersRos::receivedUDPPacket, this, _1), &m_communication_settings);
  m_device->run();
  readTypeCodeSettings();

  if (m_use_pers_conf)
  {
    readPersistentConfig();
  }

  m_device->changeSensorSettings(m_communication_settings);
  m_initialised = true;
  ROS_INFO("Successfully launched node.");
}

void SickSafetyscannersRos::readTypeCodeSettings()
{
  ROS_INFO("Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(m_communication_settings, type_code);
  m_communication_settings.setEInterfaceType(type_code.getInterfaceType());
  m_range_min = 0.1;
  m_range_max = type_code.getMaxRange();
}

void SickSafetyscannersRos::readPersistentConfig()
{
  ROS_INFO("Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(m_communication_settings, config_data);
  m_communication_settings.setStartAngle(config_data.getStartAngle());
  m_communication_settings.setEndAngle(config_data.getEndAngle());
}

void SickSafetyscannersRos::reconfigureCallback(
  const sick_safetyscanners::SickSafetyscannersConfigurationConfig& config, const uint32_t& level)
{
  if (isInitialised())
  {
    m_communication_settings.setEnabled(config.channel_enabled);
    m_communication_settings.setPublishingFrequency(skipToPublishFrequency(config.skip));
    if (config.angle_start == config.angle_end)
    {
      m_communication_settings.setStartAngle(sick::radToDeg(0));
      m_communication_settings.setEndAngle(sick::radToDeg(0));
    }
    else
    {
      m_communication_settings.setStartAngle(sick::radToDeg(config.angle_start) - m_angle_offset);
      m_communication_settings.setEndAngle(sick::radToDeg(config.angle_end) - m_angle_offset);
    }
    m_communication_settings.setFeatures(config.general_system_state,
                                         config.derived_settings,
                                         config.measurement_data,
                                         config.intrusion_data,
                                         config.application_io_data);
    m_device->changeSensorSettings(m_communication_settings);

    m_frame_id = config.frame_id;

    m_time_offset = config.time_offset;

    m_expected_frequency = config.expected_frequency;

    m_min_intensities = config.min_intensities;
  }
}

bool SickSafetyscannersRos::isInitialised()
{
  return m_initialised;
}


SickSafetyscannersRos::~SickSafetyscannersRos() {}

bool SickSafetyscannersRos::readParameters()
{
  std::string sensor_ip_adress = "192.168.1.10";
  if (!m_private_nh.getParam("sensor_ip", sensor_ip_adress))
  {
    //    sensor_ip_adress = sick_safetyscanners::SickSafetyscannersConfiguration_sensor_ip;
    ROS_WARN("Using default sensor IP: %s", sensor_ip_adress.c_str());
  }
  m_communication_settings.setSensorIp(sensor_ip_adress);


  std::string host_ip_adress = "192.168.1.9";
  if (!m_private_nh.getParam("host_ip", host_ip_adress))
  {
    ROS_WARN("Using default host IP: %s", host_ip_adress.c_str());
  }
  m_communication_settings.setHostIp(host_ip_adress);

  int host_udp_port = 0;
  if (!m_private_nh.getParam("host_udp_port", host_udp_port))
  {
    ROS_WARN("Using default host UDP Port: %i", host_udp_port);
  }
  m_communication_settings.setHostUdpPort(host_udp_port);

  ROS_WARN("If not further specified the default values for the dynamic reconfigurable parameters "
           "will be loaded.");


  int channel = 0;
  m_private_nh.getParam("channel", channel);
  m_communication_settings.setChannel(channel);

  bool enabled;
  m_private_nh.getParam("channel_enabled", enabled);
  m_communication_settings.setEnabled(enabled);

  int skip;
  m_private_nh.getParam("skip", skip);
  m_communication_settings.setPublishingFrequency(skipToPublishFrequency(skip));

  float angle_start;
  m_private_nh.getParam("angle_start", angle_start);

  float angle_end;
  m_private_nh.getParam("angle_end", angle_end);

  m_private_nh.getParam("frequency_tolerance", m_frequency_tolerance);
  m_private_nh.getParam("expected_frequency", m_expected_frequency);
  m_private_nh.getParam("timestamp_min_acceptable", m_timestamp_min_acceptable);
  m_private_nh.getParam("timestamp_max_acceptable", m_timestamp_max_acceptable);

  // Included check before calculations to prevent rounding errors while calculating
  if (angle_start == angle_end)
  {
    m_communication_settings.setStartAngle(sick::radToDeg(0));
    m_communication_settings.setEndAngle(sick::radToDeg(0));
  }
  else
  {
    m_communication_settings.setStartAngle(sick::radToDeg(angle_start) - m_angle_offset);
    m_communication_settings.setEndAngle(sick::radToDeg(angle_end) - m_angle_offset);
  }

  m_private_nh.getParam("time_offset", m_time_offset);

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

  m_communication_settings.setFeatures(
    general_system_state, derived_settings, measurement_data, intrusion_data, application_io_data);

  m_private_nh.getParam("frame_id", m_frame_id);

  m_private_nh.getParam("use_persistent_config", m_use_pers_conf);

  m_private_nh.getParam("min_intensities", m_min_intensities);

  return true;
}

void SickSafetyscannersRos::receivedUDPPacket(const sick::datastructure::Data& data)
{
  if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
  {
    sensor_msgs::LaserScan scan = createLaserScanMessage(data);

    m_diagnosed_laser_scan_publisher->publish(scan);
  }


  if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
  {
    sick_safetyscanners::ExtendedLaserScanMsg extended_scan = createExtendedLaserScanMessage(data);

    m_extended_laser_scan_publisher.publish(extended_scan);

    sick_safetyscanners::OutputPathsMsg output_paths = createOutputPathsMessage(data);
    m_output_path_publisher.publish(output_paths);
  }

  m_last_raw_data = createRawDataMessage(data);
  m_raw_data_publisher.publish(m_last_raw_data);

  m_diagnostic_updater.update();
}

std::string boolToString(bool b)
{
  return b ? "true" : "false";
}

void SickSafetyscannersRos::sensorDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  const sick_safetyscanners::DataHeaderMsg& header = m_last_raw_data.header;
  if (header.timestamp_time == 0 && header.timestamp_date == 0)
  {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::STALE,
                              "Could not get sensor state");
    return;
  }

  diagnostic_status.addf("Version version", "%c", header.version_version);
  diagnostic_status.addf("Version major version", "%u", header.version_major_version);
  diagnostic_status.addf("Version minor version", "%u", header.version_minor_version);
  diagnostic_status.addf("Version release", "%u", header.version_release);
  diagnostic_status.addf("Serial number of device", "%u", header.serial_number_of_device);
  diagnostic_status.addf(
    "Serial number of channel plug", "%u", header.serial_number_of_channel_plug);
  diagnostic_status.addf("Channel number", "%u", header.channel_number);
  diagnostic_status.addf("Sequence number", "%u", header.sequence_number);
  diagnostic_status.addf("Scan number", "%u", header.scan_number);
  diagnostic_status.addf("Timestamp date", "%u", header.timestamp_date);
  diagnostic_status.addf("Timestamp time", "%u", header.timestamp_time);

  const sick_safetyscanners::GeneralSystemStateMsg& state = m_last_raw_data.general_system_state;
  diagnostic_status.add("Run mode active", boolToString(state.run_mode_active));
  diagnostic_status.add("Standby mode active", boolToString(state.standby_mode_active));
  diagnostic_status.add("Contamination warning", boolToString(state.contamination_warning));
  diagnostic_status.add("Contamination error", boolToString(state.contamination_error));
  diagnostic_status.add("Reference contour status", boolToString(state.reference_contour_status));
  diagnostic_status.add("Manipulation status", boolToString(state.manipulation_status));
  diagnostic_status.addf(
    "Current monitoring case no table 1", "%u", state.current_monitoring_case_no_table_1);
  diagnostic_status.addf(
    "Current monitoring case no table 2", "%u", state.current_monitoring_case_no_table_2);
  diagnostic_status.addf(
    "Current monitoring case no table 3", "%u", state.current_monitoring_case_no_table_3);
  diagnostic_status.addf(
    "Current monitoring case no table 4", "%u", state.current_monitoring_case_no_table_4);
  diagnostic_status.add("Application error", boolToString(state.application_error));
  diagnostic_status.add("Device error", boolToString(state.device_error));

  if (state.device_error)
  {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Device error");
  }
  else
  {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
}

sick_safetyscanners::ExtendedLaserScanMsg
SickSafetyscannersRos::createExtendedLaserScanMessage(const sick::datastructure::Data& data)
{
  sensor_msgs::LaserScan scan = createLaserScanMessage(data);
  sick_safetyscanners::ExtendedLaserScanMsg msg;
  msg.laser_scan = scan;

  std::vector<sick::datastructure::ScanPoint> scan_points =
    data.getMeasurementDataPtr()->getScanPointsVector();
  uint32_t num_scan_points = scan_points.size();


  msg.reflektor_status.resize(num_scan_points);
  msg.intrusion.resize(num_scan_points);
  msg.reflektor_median.resize(num_scan_points);
  std::vector<bool> medians = getMedianReflectors(scan_points);
  for (uint32_t i = 0; i < num_scan_points; ++i)
  {
    const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    msg.reflektor_status[i]                         = scan_point.getReflectorBit();
    msg.intrusion[i]                                = scan_point.getContaminationBit();
    msg.reflektor_median[i]                         = medians.at(i);
  }
  return msg;
}

std::vector<bool> SickSafetyscannersRos::getMedianReflectors(
  const std::vector<sick::datastructure::ScanPoint> scan_points)
{
  std::vector<bool> res;
  res.resize(scan_points.size());
  bool last = false;
  int start = -1;
  for (size_t i = 0; i < scan_points.size(); i++)
  {
    res.at(i) = false;
    if (!last && scan_points.at(i).getReflectorBit())
    {
      last  = true;
      start = i;
    }
    else if (last && (!scan_points.at(i).getReflectorBit() || i == scan_points.size() - 1))
    {
      last                              = false;
      res.at(start + ((i - start) / 2)) = true;
    }
  }

  return res;
}

sensor_msgs::LaserScan
SickSafetyscannersRos::createLaserScanMessage(const sick::datastructure::Data& data)
{
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = m_frame_id;
  scan.header.stamp    = ros::Time::now();
  // Add time offset (to account for network latency etc.)
  scan.header.stamp += ros::Duration().fromSec(m_time_offset);
  // TODO check why returned number of beams is misaligned to size of vector
  std::vector<sick::datastructure::ScanPoint> scan_points =
    data.getMeasurementDataPtr()->getScanPointsVector();
  uint32_t num_scan_points = scan_points.size();

  scan.angle_min = sick::degToRad(data.getDerivedValuesPtr()->getStartAngle() + m_angle_offset);
  double angle_max =
    sick::degToRad(data.getMeasurementDataPtr()
                     ->getScanPointsVector()
                     .at(data.getMeasurementDataPtr()->getScanPointsVector().size() - 1)
                     .getAngle() +
                   m_angle_offset);
  scan.angle_max       = angle_max;
  scan.angle_increment = sick::degToRad(data.getDerivedValuesPtr()->getAngularBeamResolution());
  boost::posix_time::microseconds time_increment =
    boost::posix_time::microseconds(data.getDerivedValuesPtr()->getInterbeamPeriod());
  scan.time_increment = time_increment.total_microseconds() * 1e-6;
  boost::posix_time::milliseconds scan_time =
    boost::posix_time::milliseconds(data.getDerivedValuesPtr()->getScanTime());
  scan.scan_time = scan_time.total_microseconds() * 1e-6;
  scan.range_min = m_range_min;
  scan.range_max = m_range_max;
  scan.ranges.resize(num_scan_points);
  scan.intensities.resize(num_scan_points);


  for (uint32_t i = 0; i < num_scan_points; ++i)
  {
    const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    // Filter for intensities
    if (m_min_intensities < static_cast<double>(scan_point.getReflectivity()))
    {
      scan.ranges[i] = static_cast<float>(scan_point.getDistance()) *
                       data.getDerivedValuesPtr()->getMultiplicationFactor() * 1e-3; // mm -> m
      // Set values close to/greater than max range to infinity according to REP 117
      // https://www.ros.org/reps/rep-0117.html
      if (scan.ranges[i] >= (0.999 * m_range_max))
      {
        scan.ranges[i] = std::numeric_limits<double>::infinity();
      }
    }
    else
    {
      scan.ranges[i] = std::numeric_limits<double>::infinity();
    }
    scan.intensities[i] = static_cast<float>(scan_point.getReflectivity());
  }
  return scan;
}

sick_safetyscanners::OutputPathsMsg
SickSafetyscannersRos::createOutputPathsMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::OutputPathsMsg msg;

  std::shared_ptr<sick::datastructure::ApplicationData> app_data = data.getApplicationDataPtr();
  sick::datastructure::ApplicationOutputs outputs                = app_data->getOutputs();

  std::vector<bool> eval_out         = outputs.getEvalOutVector();
  std::vector<bool> eval_out_is_safe = outputs.getEvalOutIsSafeVector();
  std::vector<bool> eval_out_valid   = outputs.getEvalOutIsValidVector();

  std::vector<uint16_t> monitoring_case_numbers  = outputs.getMonitoringCaseVector();
  std::vector<bool> monitoring_case_number_flags = outputs.getMonitoringCaseFlagsVector();

  // Fix according to issue #46, however why this appears is not clear
  if (monitoring_case_number_flags.size() > 0)
  {
    msg.active_monitoring_case = monitoring_case_numbers.at(0);
  }
  else
  {
    msg.active_monitoring_case = 0;
  }

  for (size_t i = 0; i < eval_out.size(); i++)
  {
    msg.status.push_back(eval_out.at(i));
    msg.is_safe.push_back(eval_out_is_safe.at(i));
    msg.is_valid.push_back(eval_out_valid.at(i));
  }
  return msg;
}

sick_safetyscanners::RawMicroScanDataMsg
SickSafetyscannersRos::createRawDataMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::RawMicroScanDataMsg msg;

  msg.header               = createDataHeaderMessage(data);
  msg.derived_values       = createDerivedValuesMessage(data);
  msg.general_system_state = createGeneralSystemStateMessage(data);
  msg.measurement_data     = createMeasurementDataMessage(data);
  msg.intrusion_data       = createIntrusionDataMessage(data);
  msg.application_data     = createApplicationDataMessage(data);

  return msg;
}

sick_safetyscanners::DataHeaderMsg
SickSafetyscannersRos::createDataHeaderMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::DataHeaderMsg msg;

  if (!data.getDataHeaderPtr()->isEmpty())
  {
    std::shared_ptr<sick::datastructure::DataHeader> data_header = data.getDataHeaderPtr();

    msg.version_version       = data_header->getVersionIndicator();
    msg.version_release       = data_header->getVersionRelease();
    msg.version_major_version = data_header->getVersionMajorVersion();
    msg.version_minor_version = data_header->getVersionMinorVersion();

    msg.scan_number     = data_header->getScanNumber();
    msg.sequence_number = data_header->getSequenceNumber();

    msg.serial_number_of_device       = data_header->getSerialNumberOfDevice();
    msg.serial_number_of_channel_plug = data_header->getSerialNumberOfSystemPlug();

    msg.channel_number = data_header->getChannelNumber();

    msg.timestamp_date = data_header->getTimestampDate();
    msg.timestamp_time = data_header->getTimestampTime();
  }
  return msg;
}

sick_safetyscanners::DerivedValuesMsg
SickSafetyscannersRos::createDerivedValuesMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::DerivedValuesMsg msg;

  if (!data.getDerivedValuesPtr()->isEmpty())
  {
    std::shared_ptr<sick::datastructure::DerivedValues> derived_values = data.getDerivedValuesPtr();

    msg.multiplication_factor   = derived_values->getMultiplicationFactor();
    msg.scan_time               = derived_values->getScanTime();
    msg.interbeam_period        = derived_values->getInterbeamPeriod();
    msg.number_of_beams         = derived_values->getNumberOfBeams();
    msg.start_angle             = derived_values->getStartAngle() + m_angle_offset;
    msg.angular_beam_resolution = derived_values->getAngularBeamResolution();
  }
  return msg;
}

sick_safetyscanners::GeneralSystemStateMsg
SickSafetyscannersRos::createGeneralSystemStateMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::GeneralSystemStateMsg msg;

  if (!data.getGeneralSystemStatePtr()->isEmpty())
  {
    std::shared_ptr<sick::datastructure::GeneralSystemState> general_system_state =
      data.getGeneralSystemStatePtr();

    msg.run_mode_active          = general_system_state->getRunModeActive();
    msg.standby_mode_active      = general_system_state->getStandbyModeActive();
    msg.contamination_warning    = general_system_state->getContaminationWarning();
    msg.contamination_error      = general_system_state->getContaminationError();
    msg.reference_contour_status = general_system_state->getReferenceContourStatus();
    msg.manipulation_status      = general_system_state->getManipulationStatus();

    std::vector<bool> safe_cut_off_path = general_system_state->getSafeCutOffPathVector();
    for (size_t i = 0; i < safe_cut_off_path.size(); i++)
    {
      msg.safe_cut_off_path.push_back(safe_cut_off_path.at(i));
    }

    std::vector<bool> non_safe_cut_off_path = general_system_state->getNonSafeCutOffPathVector();
    for (size_t i = 0; i < non_safe_cut_off_path.size(); i++)
    {
      msg.non_safe_cut_off_path.push_back(non_safe_cut_off_path.at(i));
    }

    std::vector<bool> reset_required_cut_off_path =
      general_system_state->getResetRequiredCutOffPathVector();
    for (size_t i = 0; i < reset_required_cut_off_path.size(); i++)
    {
      msg.reset_required_cut_off_path.push_back(reset_required_cut_off_path.at(i));
    }

    msg.current_monitoring_case_no_table_1 =
      general_system_state->getCurrentMonitoringCaseNoTable1();
    msg.current_monitoring_case_no_table_2 =
      general_system_state->getCurrentMonitoringCaseNoTable2();
    msg.current_monitoring_case_no_table_3 =
      general_system_state->getCurrentMonitoringCaseNoTable3();
    msg.current_monitoring_case_no_table_4 =
      general_system_state->getCurrentMonitoringCaseNoTable4();

    msg.application_error = general_system_state->getApplicationError();
    msg.device_error      = general_system_state->getDeviceError();
  }
  return msg;
}

sick_safetyscanners::MeasurementDataMsg
SickSafetyscannersRos::createMeasurementDataMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::MeasurementDataMsg msg;

  if (!data.getMeasurementDataPtr()->isEmpty())
  {
    msg.number_of_beams = data.getMeasurementDataPtr()->getNumberOfBeams();
    msg.scan_points     = createScanPointMessageVector(data);
  }
  return msg;
}

std::vector<sick_safetyscanners::ScanPointMsg>
SickSafetyscannersRos::createScanPointMessageVector(const sick::datastructure::Data& data)
{
  std::vector<sick_safetyscanners::ScanPointMsg> msg_vector;

  std::shared_ptr<sick::datastructure::MeasurementData> measurement_data =
    data.getMeasurementDataPtr();
  std::vector<sick::datastructure::ScanPoint> scan_points = measurement_data->getScanPointsVector();
  // uint32_t num_points                                     = measurement_data->getNumberOfBeams();
  uint32_t num_points = scan_points.size();
  for (uint32_t i = 0; i < num_points; i++)
  {
    sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    sick_safetyscanners::ScanPointMsg msg;
    msg.distance              = scan_point.getDistance();
    msg.reflectivity          = scan_point.getReflectivity();
    msg.angle                 = scan_point.getAngle() + m_angle_offset;
    msg.valid                 = scan_point.getValidBit();
    msg.infinite              = scan_point.getInfiniteBit();
    msg.glare                 = scan_point.getGlareBit();
    msg.reflector             = scan_point.getReflectorBit();
    msg.contamination_warning = scan_point.getContaminationWarningBit();
    msg.contamination         = scan_point.getContaminationBit();

    msg_vector.push_back(msg);
  }
  return msg_vector;
}

sick_safetyscanners::IntrusionDataMsg
SickSafetyscannersRos::createIntrusionDataMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::IntrusionDataMsg msg;

  if (!data.getIntrusionDataPtr()->isEmpty())
  {
    msg.data = createIntrusionDatumMessageVector(data);
  }
  return msg;
}

std::vector<sick_safetyscanners::IntrusionDatumMsg>
SickSafetyscannersRos::createIntrusionDatumMessageVector(const sick::datastructure::Data& data)
{
  std::vector<sick_safetyscanners::IntrusionDatumMsg> msg_vector;

  std::shared_ptr<sick::datastructure::IntrusionData> intrusion_data = data.getIntrusionDataPtr();
  std::vector<sick::datastructure::IntrusionDatum> intrusion_datums =
    intrusion_data->getIntrusionDataVector();

  for (size_t i = 0; i < intrusion_datums.size(); i++)
  {
    sick_safetyscanners::IntrusionDatumMsg msg;
    sick::datastructure::IntrusionDatum intrusion_datum = intrusion_datums.at(i);
    msg.size                                            = intrusion_datum.getSize();
    std::vector<bool> flags                             = intrusion_datum.getFlagsVector();
    for (size_t j = 0; j < flags.size(); j++)
    {
      msg.flags.push_back(flags.at(j));
    }
    msg_vector.push_back(msg);
  }
  return msg_vector;
}

sick_safetyscanners::ApplicationDataMsg
SickSafetyscannersRos::createApplicationDataMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::ApplicationDataMsg msg;

  if (!data.getApplicationDataPtr()->isEmpty())
  {
    msg.inputs  = createApplicationInputsMessage(data);
    msg.outputs = createApplicationOutputsMessage(data);
  }
  return msg;
}

sick_safetyscanners::ApplicationInputsMsg
SickSafetyscannersRos::createApplicationInputsMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::ApplicationInputsMsg msg;

  std::shared_ptr<sick::datastructure::ApplicationData> app_data = data.getApplicationDataPtr();
  sick::datastructure::ApplicationInputs inputs                  = app_data->getInputs();
  std::vector<bool> unsafe_inputs       = inputs.getUnsafeInputsInputSourcesVector();
  std::vector<bool> unsafe_inputs_flags = inputs.getUnsafeInputsFlagsVector();
  for (size_t i = 0; i < unsafe_inputs.size(); i++)
  {
    msg.unsafe_inputs_input_sources.push_back(unsafe_inputs.at(i));
    msg.unsafe_inputs_flags.push_back(unsafe_inputs_flags.at(i));
  }
  std::vector<uint16_t> monitoring_case   = inputs.getMonitoringCasevector();
  std::vector<bool> monitoring_case_flags = inputs.getMonitoringCaseFlagsVector();
  for (size_t i = 0; i < monitoring_case.size(); i++)
  {
    msg.monitoring_case_number_inputs.push_back(monitoring_case.at(i));
    msg.monitoring_case_number_inputs_flags.push_back(monitoring_case_flags.at(i));
  }
  msg.linear_velocity_inputs_velocity_0                    = inputs.getVelocity0();
  msg.linear_velocity_inputs_velocity_0_transmitted_safely = inputs.getVelocity0TransmittedSafely();
  msg.linear_velocity_inputs_velocity_0_valid              = inputs.getVelocity0Valid();
  msg.linear_velocity_inputs_velocity_1                    = inputs.getVelocity1();
  msg.linear_velocity_inputs_velocity_1_transmitted_safely = inputs.getVelocity1TransmittedSafely();
  msg.linear_velocity_inputs_velocity_1_valid              = inputs.getVelocity1Valid();

  msg.sleep_mode_input = inputs.getSleepModeInput();

  return msg;
}

sick_safetyscanners::ApplicationOutputsMsg
SickSafetyscannersRos::createApplicationOutputsMessage(const sick::datastructure::Data& data)
{
  sick_safetyscanners::ApplicationOutputsMsg msg;

  std::shared_ptr<sick::datastructure::ApplicationData> app_data = data.getApplicationDataPtr();
  sick::datastructure::ApplicationOutputs outputs                = app_data->getOutputs();

  std::vector<bool> eval_out         = outputs.getEvalOutVector();
  std::vector<bool> eval_out_is_safe = outputs.getEvalOutIsSafeVector();
  std::vector<bool> eval_out_valid   = outputs.getEvalOutIsValidVector();
  for (size_t i = 0; i < eval_out.size(); i++)
  {
    msg.evaluation_path_outputs_eval_out.push_back(eval_out.at(i));
    msg.evaluation_path_outputs_is_safe.push_back(eval_out_is_safe.at(i));
    msg.evaluation_path_outputs_is_valid.push_back(eval_out_valid.at(i));
  }

  std::vector<uint16_t> monitoring_case   = outputs.getMonitoringCaseVector();
  std::vector<bool> monitoring_case_flags = outputs.getMonitoringCaseFlagsVector();
  for (size_t i = 0; i < monitoring_case.size(); i++)
  {
    msg.monitoring_case_number_outputs.push_back(monitoring_case.at(i));
    msg.monitoring_case_number_outputs_flags.push_back(monitoring_case_flags.at(i));
  }

  msg.sleep_mode_output       = outputs.getSleepModeOutput();
  msg.sleep_mode_output_valid = outputs.getFlagsSleepModeOutputIsValid();

  msg.error_flag_contamination_warning      = outputs.getHostErrorFlagContaminationWarning();
  msg.error_flag_contamination_error        = outputs.getHostErrorFlagContaminationError();
  msg.error_flag_manipulation_error         = outputs.getHostErrorFlagManipulationError();
  msg.error_flag_glare                      = outputs.getHostErrorFlagGlare();
  msg.error_flag_reference_contour_intruded = outputs.getHostErrorFlagReferenceContourIntruded();
  msg.error_flag_critical_error             = outputs.getHostErrorFlagCriticalError();
  msg.error_flags_are_valid                 = outputs.getFlagsHostErrorFlagsAreValid();

  msg.linear_velocity_outputs_velocity_0 = outputs.getVelocity0();
  msg.linear_velocity_outputs_velocity_0_transmitted_safely =
    outputs.getVelocity0TransmittedSafely();
  msg.linear_velocity_outputs_velocity_0_valid = outputs.getVelocity0Valid();
  msg.linear_velocity_outputs_velocity_1       = outputs.getVelocity1();
  msg.linear_velocity_outputs_velocity_1_transmitted_safely =
    outputs.getVelocity1TransmittedSafely();
  msg.linear_velocity_outputs_velocity_1_valid = outputs.getVelocity1Valid();

  std::vector<int16_t> resulting_velocities    = outputs.getResultingVelocityVector();
  std::vector<bool> resulting_velocities_flags = outputs.getResultingVelocityIsValidVector();

  for (size_t i = 0; i < resulting_velocities.size(); i++)
  {
    msg.resulting_velocity.push_back(resulting_velocities.at(i));
    msg.resulting_velocity_flags.push_back(resulting_velocities_flags.at(i));
  }


  return msg;
}

bool SickSafetyscannersRos::getFieldData(sick_safetyscanners::FieldData::Request& req,
                                         sick_safetyscanners::FieldData::Response& res)
{
  std::vector<sick::datastructure::FieldData> fields;
  m_device->requestFieldData(m_communication_settings, fields);

  for (size_t i = 0; i < fields.size(); i++)
  {
    sick::datastructure::FieldData field = fields.at(i);
    sick_safetyscanners::FieldMsg field_msg;

    field_msg.start_angle        = degToRad(field.getStartAngle() + m_angle_offset);
    field_msg.angular_resolution = degToRad(field.getAngularBeamResolution());
    field_msg.protective_field   = field.getIsProtectiveField();

    std::vector<uint16_t> ranges = field.getBeamDistances();
    for (size_t j = 0; j < ranges.size(); j++)
    {
      field_msg.ranges.push_back(static_cast<float>(ranges.at(j)) * 1e-3);
    }

    res.fields.push_back(field_msg);
  }

  datastructure::DeviceName device_name;
  m_device->requestDeviceName(m_communication_settings, device_name);
  res.device_name = device_name.getDeviceName();


  std::vector<sick::datastructure::MonitoringCaseData> monitoring_cases;
  m_device->requestMonitoringCases(m_communication_settings, monitoring_cases);

  for (size_t i = 0; i < monitoring_cases.size(); i++)
  {
    sick::datastructure::MonitoringCaseData monitoring_case_data = monitoring_cases.at(i);
    sick_safetyscanners::MonitoringCaseMsg monitoring_case_msg;

    monitoring_case_msg.monitoring_case_number = monitoring_case_data.getMonitoringCaseNumber();
    std::vector<uint16_t> mon_fields           = monitoring_case_data.getFieldIndices();
    std::vector<bool> mon_fields_valid         = monitoring_case_data.getFieldsValid();
    for (size_t j = 0; j < mon_fields.size(); j++)
    {
      monitoring_case_msg.fields.push_back(mon_fields.at(j));
      monitoring_case_msg.fields_valid.push_back(mon_fields_valid.at(j));
    }
    res.monitoring_cases.push_back(monitoring_case_msg);
  }

  return true;
}


} // namespace sick
