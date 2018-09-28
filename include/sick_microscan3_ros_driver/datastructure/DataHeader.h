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
 * \file DataHeader.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef DATAHEADER_H
#define DATAHEADER_H

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class DataHeader
{
public:
  DataHeader();

  UINT8 getVersionIndicator() const;
  void setVersionIndicator(const UINT8& version_indicator);

  UINT8 getVersionMajorVersion() const;
  void setVersionMajorVersion(const UINT8& version_major_version);

  UINT8 getVersionMinorVersion() const;
  void setVersionMinorVersion(const UINT8& version_minor_version);

  UINT8 getVersionRelease() const;
  void setVersionRelease(const UINT8& version_release);

  UINT32 getSerialNumberOfDevice() const;
  void setSerialNumberOfDevice(const UINT32& serial_number_of_device);

  UINT32 getSerialNumberOfSystemPlug() const;
  void setSerialNumberOfSystemPlug(const UINT32& serial_number_of_system_plug);

  UINT8 getChannelNumber() const;
  void setChannelNumber(const UINT8& channel_number);

  UINT32 getSequenceNumber() const;
  void setSequenceNumber(const UINT32& sequence_number);

  UINT32 getScanNumber() const;
  void setScanNumber(const UINT32& scan_number);

  UINT16 getTimestampDate() const;
  void setTimestampDate(const UINT16& timestamp_date);

  UINT32 getTimestampTime() const;
  void setTimestampTime(const UINT32& timestamp_time);

  UINT16 getGeneralSystemStateBlockOffset() const;
  void setGeneralSystemStateBlockOffset(const UINT16& general_system_state_block_offset);

  UINT16 getGeneralSystemStateBlockSize() const;
  void setGeneralSystemStateBlockSize(const UINT16& general_system_state_block_size);

  UINT16 getDerivedValuesBlockOffset() const;
  void setDerivedValuesBlockOffset(const UINT16& derived_values_block_offset);

  UINT16 getDerivedValuesBlockSize() const;
  void setDerivedValuesBlockSize(const UINT16& derived_values_block_size);

  UINT16 getMeasurementDataBlockOffset() const;
  void setMeasurementDataBlockOffset(const UINT16& measurement_data_block_offset);

  UINT16 getMeasurementDataBlockSize() const;
  void setMeasurementDataBlockSize(const UINT16& measurement_data_block_size);

  UINT16 getIntrusionDataBlockOffset() const;
  void setIntrusionDataBlockOffset(const UINT16& intrusion_data_block_offset);

  UINT16 getIntrusionDataBlockSize() const;
  void setIntrusionDataBlockSize(const UINT16& intrusion_data_block_size);

  UINT16 getApplicationDataBlockOffset() const;
  void setApplicationDataBlockOffset(const UINT16& application_data_block_offset);

  UINT16 getApplicationDataBlockSize() const;
  void setApplicationDataBlockSize(const UINT16& application_data_block_size);

  bool isEmpty() const;
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;

  UINT8 m_version_indicator;
  UINT8 m_version_major_version;
  UINT8 m_version_minor_version;
  UINT8 m_version_release;
  UINT32 m_serial_number_of_device;
  UINT32 m_serial_number_of_system_plug;
  UINT8 m_channel_number;
  UINT32 m_sequence_number;
  UINT32 m_scan_number;
  UINT16 m_timestamp_date;
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

} // namespace datastructure
} // namespace sick

#endif
