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

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_DATAHEADER_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_DATAHEADER_H

#include <stdint.h>

namespace sick {
namespace datastructure {

/*!
 * \brief Contains the content of the data header of a udp data packet.
 */
class DataHeader
{
public:
  /*!
   * \brief Constructor of an empty data header.
   */
  DataHeader();

  /*!
   * \brief Gets the version indicator. Capital letter 'V' or 'R' for releases.
   *
   * \returns The version indicator.
   */
  uint8_t getVersionIndicator() const;

  /*!
   * \brief Sets the version indicator.
   *
   * \param version_indicator The new version indicator.
   */
  void setVersionIndicator(const uint8_t& version_indicator);

  /*!
   * \brief Returns the major version number. Different numbers indicate incompatible
   * implementation.
   *
   * \returns The major version indicator.
   */
  uint8_t getVersionMajorVersion() const;

  /*!
   * \brief Sets the major version number.
   *
   * \param version_major_version The new major version number.
   */
  void setVersionMajorVersion(const uint8_t& version_major_version);

  /*!
   * \brief Returns the minor version number. Different numbers indicate compatible implementation
   * if the major version is equal.
   *
   * \returns The minor version number.
   */
  uint8_t getVersionMinorVersion() const;
  /*!
   * \brief Sets the minor version number.
   *
   * \param version_minor_version The new minor version number.
   */
  void setVersionMinorVersion(const uint8_t& version_minor_version);

  /*!
   * \brief Gets the release of the version.
   *
   * \returns The release of the version.
   */
  uint8_t getVersionRelease() const;
  /*!
   * \brief Sets the release of the version.
   *
   * \param version_release The new release of the version.
   */
  void setVersionRelease(const uint8_t& version_release);

  /*!
   * \brief Gets the serial number of the device.
   *
   * \returns The serial number of the device.
   */
  uint32_t getSerialNumberOfDevice() const;
  /*!
   * \brief Sets the serial number of the device.
   *
   * \param serial_number_of_device The serial number of the device.
   */
  void setSerialNumberOfDevice(const uint32_t& serial_number_of_device);

  /*!
   * \brief Gets the serial number of the system plug.
   *
   * \returns The serial number of the system plug.
   */
  uint32_t getSerialNumberOfSystemPlug() const;
  /*!
   * \brief Sets the serial number of the system plug.
   *
   * \param serial_number_of_system_plug The serial number of the system plug.
   */
  void setSerialNumberOfSystemPlug(const uint32_t& serial_number_of_system_plug);

  /*!
   * \brief Gets the number of channel the measurement data belongs to.
   *
   * \returns The number of channel.
   */
  uint8_t getChannelNumber() const;
  /*!
   * \brief Sets the number of channel the measurement data belongs to.
   *
   * \param channel_number The new channel number.
   */
  void setChannelNumber(const uint8_t& channel_number);

  /*!
   * \brief Gets the sequence number of the measurement data.
   *
   * This sequence number increases with each instance
   * of measurement data that is produced by a given
   * channel.
   *
   * \returns The sequence number.
   */
  uint32_t getSequenceNumber() const;
  /*!
   * \brief Sets the sequence number of the measurement data.
   *
   * \param sequence_number The sequence number.
   */
  void setSequenceNumber(const uint32_t& sequence_number);

  /*!
   * \brief Gets the scan number of the measurement data instance.
   *
   * \returns The scan number.
   */
  uint32_t getScanNumber() const;
  /*!
   * \brief Sets the scan number of the measurement data.
   *
   * \param scan_number The new scan number.
   */
  void setScanNumber(const uint32_t& scan_number);

  /*!
   * \brief Gets the timestamp date.
   *
   * A timestamp when the measurement data output instance was created.
   * Note: While the measurements are performed periodically using the configured scan cycle time,
   * you may experience a jitter in the timestamp reported here as it is not synchronized with
   * the start of the scan.
   *
   * \returns The timestamp date.
   */
  uint16_t getTimestampDate() const;
  /*!
   * \brief Sets the timestamp date.
   *
   * \param timestamp_date The new timestamp date.
   */
  void setTimestampDate(const uint16_t& timestamp_date);

  /*!
   * \brief Gets the timestamp time.
   *
   * A timestamp when the measurement data output instance was created.
   * Note: While the measurements are performed periodically using the configured scan cycle time,
   * you may experience a jitter in the timestamp reported here as it is not synchronized with
   * the start of the scan.
   *
   * \returns The timestamp time.
   */
  uint32_t getTimestampTime() const;
  /*!
   * \brief Sets the timestamp time.
   *
   * \param timestamp_time The new timestamp time.
   */
  void setTimestampTime(const uint32_t& timestamp_time);

  /*!
   * \brief Gets the general system state block offset.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The general system state block offset.
   */
  uint16_t getGeneralSystemStateBlockOffset() const;
  /*!
   * \brief Sets the general system state block offset.
   *
   * \param general_system_state_block_offset The new general system state block offset.
   */
  void setGeneralSystemStateBlockOffset(const uint16_t& general_system_state_block_offset);

  /*!
   * \brief Gets the general system state block size.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The general system state block size.
   */
  uint16_t getGeneralSystemStateBlockSize() const;
  /*!
   * \brief Sets the general system state block size.
   *
   * \param general_system_state_block_size The new general system state block size.
   */
  void setGeneralSystemStateBlockSize(const uint16_t& general_system_state_block_size);

  /*!
   * \brief Gets the derived values block offset.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The derived values block offset.
   */
  uint16_t getDerivedValuesBlockOffset() const;
  /*!
   * \brief Sets the derived values block offset.
   *
   * \param derived_values_block_offset The new derived values block offset.
   */
  void setDerivedValuesBlockOffset(const uint16_t& derived_values_block_offset);

  /*!
   * \brief Gets the derived values block size.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The derived values block size.
   */
  uint16_t getDerivedValuesBlockSize() const;
  /*!
   * \brief Sets the derived values block size.
   *
   * \param derived_values_block_size The new derived values block size.
   */
  void setDerivedValuesBlockSize(const uint16_t& derived_values_block_size);

  /*!
   * \brief Gets the measurement data block offset.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The measurement data block offset.
   */
  uint16_t getMeasurementDataBlockOffset() const;
  /*!
   * \brief Sets the measurement data block offset.
   *
   * \param measurement_data_block_offset The new measurement data block offset.
   */
  void setMeasurementDataBlockOffset(const uint16_t& measurement_data_block_offset);

  /*!
   * \brief Gets the measurement data block size.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The measurement data block size.
   */
  uint16_t getMeasurementDataBlockSize() const;
  /*!
   * \brief Sets the measurement data block size.
   *
   * \param measurement_data_block_size The new measurement data block size.
   */
  void setMeasurementDataBlockSize(const uint16_t& measurement_data_block_size);

  /*!
   * \brief Gets the intrusion data block offset.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The intrusion data block offset.
   */
  uint16_t getIntrusionDataBlockOffset() const;
  /*!
   * \brief Sets the intrusion data block offset.
   *
   * \param intrusion_data_block_offset The new intrusion data block offset.
   */
  void setIntrusionDataBlockOffset(const uint16_t& intrusion_data_block_offset);

  /*!
   * \brief Gets the intrusion data block size.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The intrusion data block size.
   */
  uint16_t getIntrusionDataBlockSize() const;
  /*!
   * \brief Sets the intrusion data block size.
   *
   * \param intrusion_data_block_size The new intrusion data block size.
   */
  void setIntrusionDataBlockSize(const uint16_t& intrusion_data_block_size);

  /*!
   * \brief Gets the application io data block offset.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The application io data block offset.
   */
  uint16_t getApplicationDataBlockOffset() const;
  /*!
   * \brief Sets the application io data block offset.
   *
   * \param application_data_block_offset The new application io data block offset.
   */
  void setApplicationDataBlockOffset(const uint16_t& application_data_block_offset);

  /*!
   * \brief Gets the application io data block size.
   *
   * If a block is  not included (not configured or data currently not available) then Size = 0
   * and Offset = 0.
   *
   * \returns The application data io offset.
   */
  uint16_t getApplicationDataBlockSize() const;
  /*!
   * \brief Sets the application io data block size.
   *
   * \param application_data_block_size The new application data block size.
   */
  void setApplicationDataBlockSize(const uint16_t& application_data_block_size);

  /*!
   * \brief Checks if the data header block is empty.
   *
   * \returns If the data header is empty.
   */
  bool isEmpty() const;
  /*!
   * \brief Sets if the data header is empty.
   *
   * \param is_empty If the data header is empty.
   */
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

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_DATAHEADER_H
