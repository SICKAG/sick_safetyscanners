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

#include <stdint.h>

namespace sick {
namespace datastructure {

class DataHeader
{
public:
  DataHeader();

  uint8_t getVersionIndicator() const;
  void setVersionIndicator(const uint8_t& version_indicator);

  uint8_t getVersionMajorVersion() const;
  void setVersionMajorVersion(const uint8_t& version_major_version);

  uint8_t getVersionMinorVersion() const;
  void setVersionMinorVersion(const uint8_t& version_minor_version);

  uint8_t getVersionRelease() const;
  void setVersionRelease(const uint8_t& version_release);

  uint32_t getSerialNumberOfDevice() const;
  void setSerialNumberOfDevice(const uint32_t& serial_number_of_device);

  uint32_t getSerialNumberOfSystemPlug() const;
  void setSerialNumberOfSystemPlug(const uint32_t& serial_number_of_system_plug);

  uint8_t getChannelNumber() const;
  void setChannelNumber(const uint8_t& channel_number);

  uint32_t getSequenceNumber() const;
  void setSequenceNumber(const uint32_t& sequence_number);

  uint32_t getScanNumber() const;
  void setScanNumber(const uint32_t& scan_number);

  uint16_t getTimestampDate() const;
  void setTimestampDate(const uint16_t& timestamp_date);

  uint32_t getTimestampTime() const;
  void setTimestampTime(const uint32_t& timestamp_time);

  uint16_t getGeneralSystemStateBlockOffset() const;
  void setGeneralSystemStateBlockOffset(const uint16_t& general_system_state_block_offset);

  uint16_t getGeneralSystemStateBlockSize() const;
  void setGeneralSystemStateBlockSize(const uint16_t& general_system_state_block_size);

  uint16_t getDerivedValuesBlockOffset() const;
  void setDerivedValuesBlockOffset(const uint16_t& derived_values_block_offset);

  uint16_t getDerivedValuesBlockSize() const;
  void setDerivedValuesBlockSize(const uint16_t& derived_values_block_size);

  uint16_t getMeasurementDataBlockOffset() const;
  void setMeasurementDataBlockOffset(const uint16_t& measurement_data_block_offset);

  uint16_t getMeasurementDataBlockSize() const;
  void setMeasurementDataBlockSize(const uint16_t& measurement_data_block_size);

  uint16_t getIntrusionDataBlockOffset() const;
  void setIntrusionDataBlockOffset(const uint16_t& intrusion_data_block_offset);

  uint16_t getIntrusionDataBlockSize() const;
  void setIntrusionDataBlockSize(const uint16_t& intrusion_data_block_size);

  uint16_t getApplicationDataBlockOffset() const;
  void setApplicationDataBlockOffset(const uint16_t& application_data_block_offset);

  uint16_t getApplicationDataBlockSize() const;
  void setApplicationDataBlockSize(const uint16_t& application_data_block_size);

  bool isEmpty() const;
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;

  uint8_t m_version_indicator;
  uint8_t m_version_major_version;
  uint8_t m_version_minor_version;
  uint8_t m_version_release;
  uint32_t m_serial_number_of_device;
  uint32_t m_serial_number_of_system_plug;
  uint8_t m_channel_number;
  uint32_t m_sequence_number;
  uint32_t m_scan_number;
  uint16_t m_timestamp_date;
  uint32_t m_timestamp_time;
  uint16_t m_general_system_state_block_offset;
  uint16_t m_general_system_state_block_size;
  uint16_t m_derived_values_block_offset;
  uint16_t m_derived_values_block_size;
  uint16_t m_measurement_data_block_offset;
  uint16_t m_measurement_data_block_size;
  uint16_t m_intrusion_data_block_offset;
  uint16_t m_intrusion_data_block_size;
  uint16_t m_application_data_block_offset;
  uint16_t m_application_data_block_size;
};

} // namespace datastructure
} // namespace sick

#endif
