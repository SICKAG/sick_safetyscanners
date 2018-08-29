#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class DataHeader
{
public:
  DataHeader();

  UINT8 getVersionIndicator() const;
  void setVersionIndicator(const UINT8 &version_indicator);

  UINT8 getVersionMajorVersion() const;
  void setVersionMajorVersion(const UINT8 &version_major_version);

  UINT8 getVersionMinorVersion() const;
  void setVersionMinorVersion(const UINT8 &version_minor_version);

  UINT8 getVersionRelease() const;
  void setVersionRelease(const UINT8 &version_release);

  UINT32 getSerialNumberOfDevice() const;
  void setSerialNumberOfDevice(const UINT32 &serial_number_of_device);

  UINT32 getSerialNumberOfSystemPlug() const;
  void setSerialNumberOfSystemPlug(const UINT32 &serial_number_of_system_plug);

  UINT8 getChannelNumber() const;
  void setChannelNumber(const UINT8 &channel_number);

  UINT32 getSequenceNumber() const;
  void setSequenceNumber(const UINT32 &sequence_number);

  UINT32 getScanNumber() const;
  void setScanNumber(const UINT32 &scan_number);

  UINT16 getTimestampDate() const;
  void setTimestampDate(const UINT16 &timestamp_date);

  UINT32 getTimestampTime() const;
  void setTimestampTime(const UINT32 &timestamp_time);

  UINT16 getGeneralSystemStateBlockOffset() const;
  void setGeneralSystemStateBlockOffset(const UINT16 &general_system_state_block_offset);

  UINT16 getGeneralSystemStateBlockSize() const;
  void setGeneralSystemStateBlockSize(const UINT16 &general_system_state_block_size);

  UINT16 getDerivedValuesBlockOffset() const;
  void setDerivedValuesBlockOffset(const UINT16 &derived_values_block_offset);

  UINT16 getDerivedValuesBlockSize() const;
  void setDerivedValuesBlockSize(const UINT16 &derived_values_block_size);

  UINT16 getMeasurementDataBlockOffset() const;
  void setMeasurementDataBlockOffset(const UINT16 &measurement_data_block_offset);

  UINT16 getMeasurementDataBlockSize() const;
  void setMeasurementDataBlockSize(const UINT16 &measurement_data_block_size);

  UINT16 getIntrusionDataBlockOffset() const;
  void setIntrusionDataBlockOffset(const UINT16 &intrusion_data_block_offset);

  UINT16 getIntrusionDataBlockSize() const;
  void setIntrusionDataBlockSize(const UINT16 &intrusion_data_block_size);

  UINT16 getApplicationDataBlockOffset() const;
  void setApplicationDataBlockOffset(const UINT16 &application_data_block_offset);

  UINT16 getApplicationDataBlockSize() const;
  void setApplicationDataBlockSize(const UINT16 &application_data_block_size);

private:
  UINT8 m_version_indicator;
  UINT8 m_version_major_version;
  UINT8 m_version_minor_version;
  UINT8 m_version_release;
  UINT32 m_serial_number_of_device;
  UINT32 m_serial_number_of_system_plug;
  UINT8 m_channel_number;
  //3 Byte reserved
  UINT32 m_sequence_number;
  UINT32 m_scan_number;
  UINT16 m_timestamp_date;
  // 2 BYte reserved
  UINT32 m_timestamp_time;
  UINT16 m_general_system_state_block_offset;
  UINT16 m_general_system_state_block_size;
  UINT16 m_derived_values_block_offset;
  UINT16 m_derived_values_block_size;
  UINT16 m_measurement_data_block_offset;
  UINT16 m_measurement_data_block_size;
  UINT16 m_intrusion_data_block_offset;
  UINT16 m_intrusion_data_block_size;
  UINT16 m_application_data_block_offset;
  UINT16 m_application_data_block_size;

};

}
}
