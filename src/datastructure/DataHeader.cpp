#include <sick_microscan3_ros_driver/datastructure/DataHeader.h>

namespace sick {
namespace datastructure {

DataHeader::DataHeader()
{

}

UINT8 DataHeader::getVersionIndicator() const
{
  return m_version_indicator;
}

void DataHeader::setVersionIndicator(const UINT8 &version_indicator)
{
  m_version_indicator = version_indicator;
}

UINT8 DataHeader::getVersionMajorVersion() const
{
  return m_version_major_version;
}

void DataHeader::setVersionMajorVersion(const UINT8 &version_major_version)
{
  m_version_major_version = version_major_version;
}

UINT8 DataHeader::getVersionMinorVersion() const
{
  return m_version_minor_version;
}

void DataHeader::setVersionMinorVersion(const UINT8 &version_minor_version)
{
  m_version_minor_version = version_minor_version;
}

UINT8 DataHeader::getVersionRelease() const
{
  return m_version_release;
}

void DataHeader::setVersionRelease(const UINT8 &version_release)
{
  m_version_release = version_release;
}

UINT32 DataHeader::getSerialNumberOfDevice() const
{
  return m_serial_number_of_device;
}

void DataHeader::setSerialNumberOfDevice(const UINT32 &serial_number_of_device)
{
  m_serial_number_of_device = serial_number_of_device;
}

UINT32 DataHeader::getSerialNumberOfSystemPlug() const
{
  return m_serial_number_of_system_plug;
}

void DataHeader::setSerialNumberOfSystemPlug(const UINT32 &serial_number_of_system_plug)
{
  m_serial_number_of_system_plug = serial_number_of_system_plug;
}

UINT8 DataHeader::getChannelNumber() const
{
  return m_channel_number;
}

void DataHeader::setChannelNumber(const UINT8 &channel_number)
{
  m_channel_number = channel_number;
}

UINT32 DataHeader::getSequenceNumber() const
{
  return m_sequence_number;
}

void DataHeader::setSequenceNumber(const UINT32 &sequence_number)
{
  m_sequence_number = sequence_number;
}

UINT32 DataHeader::getScanNumber() const
{
  return m_scan_number;
}

void DataHeader::setScanNumber(const UINT32 &scan_number)
{
  m_scan_number = scan_number;
}

UINT16 DataHeader::getTimestampDate() const
{
  return m_timestamp_date;
}

void DataHeader::setTimestampDate(const UINT16 &timestamp_date)
{
  m_timestamp_date = timestamp_date;
}

UINT32 DataHeader::getTimestampTime() const
{
  return m_timestamp_time;
}

void DataHeader::setTimestampTime(const UINT32 &timestamp_time)
{
  m_timestamp_time = timestamp_time;
}

UINT16 DataHeader::getGeneralSystemStateBlockOffset() const
{
  return m_general_system_state_block_offset;
}

void DataHeader::setGeneralSystemStateBlockOffset(const UINT16 &general_system_state_block_offset)
{
  m_general_system_state_block_offset = general_system_state_block_offset;
}

UINT16 DataHeader::getGeneralSystemStateBlockSize() const
{
  return m_general_system_state_block_size;
}

void DataHeader::setGeneralSystemStateBlockSize(const UINT16 &general_system_state_block_size)
{
  m_general_system_state_block_size = general_system_state_block_size;
}

UINT16 DataHeader::getDerivedValuesBlockOffset() const
{
  return m_derived_values_block_offset;
}

void DataHeader::setDerivedValuesBlockOffset(const UINT16 &derived_values_block_offset)
{
  m_derived_values_block_offset = derived_values_block_offset;
}

UINT16 DataHeader::getDerivedValuesBlockSize() const
{
  return m_derived_values_block_size;
}

void DataHeader::setDerivedValuesBlockSize(const UINT16 &derived_values_block_size)
{
  m_derived_values_block_size = derived_values_block_size;
}

UINT16 DataHeader::getMeasurementDataBlockOffset() const
{
  return m_measurement_data_block_offset;
}

void DataHeader::setMeasurementDataBlockOffset(const UINT16 &measurement_data_block_offset)
{
  m_measurement_data_block_offset = measurement_data_block_offset;
}

UINT16 DataHeader::getMeasurementDataBlockSize() const
{
  return m_measurement_data_block_size;
}

void DataHeader::setMeasurementDataBlockSize(const UINT16 &measurement_data_block_size)
{
  m_measurement_data_block_size = measurement_data_block_size;
}

UINT16 DataHeader::getIntrusionDataBlockOffset() const
{
  return m_intrusion_data_block_offset;
}

void DataHeader::setIntrusionDataBlockOffset(const UINT16 &intrusion_data_block_offset)
{
  m_intrusion_data_block_offset = intrusion_data_block_offset;
}

UINT16 DataHeader::getIntrusionDataBlockSize() const
{
  return m_intrusion_data_block_size;
}

void DataHeader::setIntrusionDataBlockSize(const UINT16 &intrusion_data_block_size)
{
  m_intrusion_data_block_size = intrusion_data_block_size;
}

UINT16 DataHeader::getApplicationDataBlockOffset() const
{
  return m_application_data_block_offset;
}

void DataHeader::setApplicationDataBlockOffset(const UINT16 &application_data_block_offset)
{
  m_application_data_block_offset = application_data_block_offset;
}

UINT16 DataHeader::getApplicationDataBlockSize() const
{
  return m_application_data_block_size;
}

void DataHeader::setApplicationDataBlockSize(const UINT16 &application_data_block_size)
{
  m_application_data_block_size = application_data_block_size;
}

}
}

