// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2019, SICK AG, Waldkirch
*  Copyright (C) 2019, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file ConfigMetadata.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGMETADATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGMETADATA_H

#include <iostream>
#include <vector>


namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the serial number of a laser scanner.
 */
class ConfigMetadata
{
public:
  /*!
   * \brief Constructor of the config metadata.
   */
  ConfigMetadata();

  /*!
   * \brief Gets the version indicator for the scanner.
   *
   * \returns The version indicator for the scanner.
   */
  std::string getVersionCVersion() const;
  /*!
   * \brief Sets the version indicator for the scanner.
   *
   * \param version_c_version The version indicator for the scanner.
   */
  void setVersionCVersion(const std::string& version_c_version);

  /*!
   * \brief Gets the major version number for the scanner.
   *
   * \returns The version indicator for the scanner.
   */
  uint8_t getVersionMajorVersionNumber() const;
  /*!
   * \brief Sets the major version number for the scanner.
   *
   * \param version_major_version_number The major version number for the scanner.
   */
  void setVersionMajorVersionNumber(const uint8_t& version_major_version_number);

  /*!
   * \brief Gets the minor version number for the scanner.
   *
   * \returns The minor version number for the scanner.
   */
  uint8_t getVersionMinorVersionNumber() const;
  /*!
   * \brief Sets the minor version number for the scanner.
   *
   * \param version_minor_version_number The minor version number for the scanner.
   */
  void setVersionMinorVersionNumber(const uint8_t& version_minor_version_number);

  /*!
   * \brief Gets the version release number for the scanner.
   *
   * \returns The version release number for the scanner.
   */
  uint8_t getVersionReleaseNumber() const;
  /*!
   * \brief Sets the version release number for the scanner.
   *
   * \param version_release_number The version release number for the scanner.
   */
  void setVersionReleaseNumber(const uint8_t& version_release_number);

  /*!
   * \brief Gets the modification time date for the scanner.
   *
   * \returns The modification time date for the scanner.
   */
  uint16_t getModificationTimeDate() const;
  /*!
   * \brief Sets the modification time date for the scanner.
   *
   * \param modification_time_date The modification time date for the scanner.
   */
  void setModificationTimeDate(const uint16_t& modification_time_date);

  /*!
   * \brief Gets the modification time time for the scanner.
   *
   * \returns The modification time time for the scanner.
   */
  uint32_t getModificationTimeTime() const;
  /*!
   * \brief Sets the modification time time for the scanner.
   *
   * \param modification_time_time The modification time time for the scanner.
   */
  void setModificationTimeTime(const uint32_t& modification_time_time);

  /*!
   * \brief Gets the transfer time date for the scanner.
   *
   * \returns The transfer time date for the scanner.
   */
  uint16_t getTransferTimeDate() const;
  /*!
   * \brief Sets the transfer time date for the scanner.
   *
   * \param transfer_time_date The transfer time date for the scanner.
   */
  void setTransferTimeDate(const uint16_t& transfer_time_date);

  /*!
   * \brief Gets the transfer time time for the scanner.
   *
   * \returns The transfer time time for the scanner.
   */
  uint32_t getTransferTimeTime() const;
  /*!
   * \brief Sets the transfer time time for the scanner.
   *
   * \param transfer_time_time The transfer time time for the scanner.
   */
  void setTransferTimeTime(const uint32_t& transfer_time_time);

  /*!
   * \brief Gets the application checksum for the scanner.
   *
   * \returns The application checksum for the scanner.
   */
  uint32_t getAppChecksum() const;
  /*!
   * \brief Sets the application checksum for the scanner.
   *
   * \param app_checksum The application checksum for the scanner.
   */
  void setAppChecksum(const uint32_t& app_checksum);

  /*!
   * \brief Gets the overall checksum for the scanner.
   *
   * \returns The overall checksum for the scanner.
   */
  uint32_t getOverallChecksum() const;
  /*!
   * \brief Sets the overall checksum for the scanner.
   *
   * \param overall_checksum The overall checksum for the scanner.
   */
  void setOverallChecksum(const uint32_t& overall_checksum);

  /*!
   * \brief Gets the integrity hash for the scanner.
   *
   * \returns The integrity hash for the scanner.
   */
  std::vector<uint32_t> getIntegrityHash() const;
  /*!
   * \brief Sets the integrity hash for the scanner.
   *
   * \param integrity_hash The integrity hash for the scanner.
   */
  void setIntegrityHash(const std::vector<uint32_t>& integrity_hash);


private:
  std::string m_version_c_version;
  uint8_t m_version_major_version_number;
  uint8_t m_version_minor_version_number;
  uint8_t m_version_release_number;
  uint16_t m_modification_time_date;
  uint32_t m_modification_time_time;
  uint16_t m_transfer_time_date;
  uint32_t m_transfer_time_time;
  uint32_t m_app_checksum;
  uint32_t m_overall_checksum;
  std::vector<uint32_t> m_integrity_hash;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGMETADATA_H
