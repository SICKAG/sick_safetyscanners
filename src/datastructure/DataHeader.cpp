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
 * \file DataHeader.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/DataHeader.h>

namespace sick {
namespace datastructure {

DataHeader::DataHeader()
  : m_is_empty(false)
{
}

uint8_t DataHeader::getVersionIndicator() const
{
  return m_version_indicator;
}

void DataHeader::setVersionIndicator(const uint8_t& version_indicator)
{
  m_version_indicator = version_indicator;
}

uint8_t DataHeader::getVersionMajorVersion() const
{
  return m_version_major_version;
}

void DataHeader::setVersionMajorVersion(const uint8_t& version_major_version)
{
  m_version_major_version = version_major_version;
}

uint8_t DataHeader::getVersionMinorVersion() const
{
  return m_version_minor_version;
}

void DataHeader::setVersionMinorVersion(const uint8_t& version_minor_version)
{
  m_version_minor_version = version_minor_version;
}

uint8_t DataHeader::getVersionRelease() const
{
  return m_version_release;
}

void DataHeader::setVersionRelease(const uint8_t& version_release)
{
  m_version_release = version_release;
}

uint32_t DataHeader::getSerialNumberOfDevice() const
{
  return m_serial_number_of_device;
}

void DataHeader::setSerialNumberOfDevice(const uint32_t& serial_number_of_device)
{
  m_serial_number_of_device = serial_number_of_device;
}

uint32_t DataHeader::getSerialNumberOfSystemPlug() const
{
  return m_serial_number_of_system_plug;
}

void DataHeader::setSerialNumberOfSystemPlug(const uint32_t& serial_number_of_system_plug)
{
  m_serial_number_of_system_plug = serial_number_of_system_plug;
}

uint8_t DataHeader::getChannelNumber() const
{
  return m_channel_number;
}

void DataHeader::setChannelNumber(const uint8_t& channel_number)
{
  m_channel_number = channel_number;
}

uint32_t DataHeader::getSequenceNumber() const
{
  return m_sequence_number;
}

void DataHeader::setSequenceNumber(const uint32_t& sequence_number)
{
  m_sequence_number = sequence_number;
}

uint32_t DataHeader::getScanNumber() const
{
  return m_scan_number;
}

void DataHeader::setScanNumber(const uint32_t& scan_number)
{
  m_scan_number = scan_number;
}

uint16_t DataHeader::getTimestampDate() const
{
  return m_timestamp_date;
}

void DataHeader::setTimestampDate(const uint16_t& timestamp_date)
{
  m_timestamp_date = timestamp_date;
}

uint32_t DataHeader::getTimestampTime() const
{
  return m_timestamp_time;
}

void DataHeader::setTimestampTime(const uint32_t& timestamp_time)
{
  m_timestamp_time = timestamp_time;
}

uint16_t DataHeader::getGeneralSystemStateBlockOffset() const
{
  return m_general_system_state_block_offset;
}

void DataHeader::setGeneralSystemStateBlockOffset(const uint16_t& general_system_state_block_offset)
{
  m_general_system_state_block_offset = general_system_state_block_offset;
}

uint16_t DataHeader::getGeneralSystemStateBlockSize() const
{
  return m_general_system_state_block_size;
}

void DataHeader::setGeneralSystemStateBlockSize(const uint16_t& general_system_state_block_size)
{
  m_general_system_state_block_size = general_system_state_block_size;
}

uint16_t DataHeader::getDerivedValuesBlockOffset() const
{
  return m_derived_values_block_offset;
}

void DataHeader::setDerivedValuesBlockOffset(const uint16_t& derived_values_block_offset)
{
  m_derived_values_block_offset = derived_values_block_offset;
}

uint16_t DataHeader::getDerivedValuesBlockSize() const
{
  return m_derived_values_block_size;
}

void DataHeader::setDerivedValuesBlockSize(const uint16_t& derived_values_block_size)
{
  m_derived_values_block_size = derived_values_block_size;
}

uint16_t DataHeader::getMeasurementDataBlockOffset() const
{
  return m_measurement_data_block_offset;
}

void DataHeader::setMeasurementDataBlockOffset(const uint16_t& measurement_data_block_offset)
{
  m_measurement_data_block_offset = measurement_data_block_offset;
}

uint16_t DataHeader::getMeasurementDataBlockSize() const
{
  return m_measurement_data_block_size;
}

void DataHeader::setMeasurementDataBlockSize(const uint16_t& measurement_data_block_size)
{
  m_measurement_data_block_size = measurement_data_block_size;
}

uint16_t DataHeader::getIntrusionDataBlockOffset() const
{
  return m_intrusion_data_block_offset;
}

void DataHeader::setIntrusionDataBlockOffset(const uint16_t& intrusion_data_block_offset)
{
  m_intrusion_data_block_offset = intrusion_data_block_offset;
}

uint16_t DataHeader::getIntrusionDataBlockSize() const
{
  return m_intrusion_data_block_size;
}

void DataHeader::setIntrusionDataBlockSize(const uint16_t& intrusion_data_block_size)
{
  m_intrusion_data_block_size = intrusion_data_block_size;
}

uint16_t DataHeader::getApplicationDataBlockOffset() const
{
  return m_application_data_block_offset;
}

void DataHeader::setApplicationDataBlockOffset(const uint16_t& application_data_block_offset)
{
  m_application_data_block_offset = application_data_block_offset;
}

uint16_t DataHeader::getApplicationDataBlockSize() const
{
  return m_application_data_block_size;
}

void DataHeader::setApplicationDataBlockSize(const uint16_t& application_data_block_size)
{
  m_application_data_block_size = application_data_block_size;
}

bool DataHeader::isEmpty() const
{
  return m_is_empty;
}

void DataHeader::setIsEmpty(bool is_empty)
{
  m_is_empty = is_empty;
}

} // namespace datastructure
} // namespace sick
