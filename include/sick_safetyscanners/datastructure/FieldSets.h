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
 * \file FieldSets.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_FIELDSETS_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_FIELDSETS_H

#include <iostream>
#include <vector>

namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the field name of a laser scanner.
 */
class FieldSets
{
public:
  /*!
   * \brief Constructor of the field name.
   */
  FieldSets();

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
   * \brief Gets the length of the field name.
   *
   * \returns The length of the field name.
   */
  std::vector<uint32_t> getNameLength() const;
  /*!
   * \brief Sets the length of the field name.
   *
   * \param name_length The length of the field name.
   */
  void setNameLength(const std::vector<uint32_t>& name_length);
  /*!
   * \brief Gets the field name for the scanner.
   *
   * \returns The field name for the scanner.
   */
  std::vector<std::string> getFieldName() const;
  /*!
   * \brief Sets the field name for the scanner.
   *
   * \param field_name The field name for the scanner.
   */
  void setFieldName(const std::vector<std::string>& field_name);

  /*!
   * \brief Gets if the fields are defined.
   *
   * \returns if the fields are defined.
   */
  std::vector<bool> getIsDefined() const;
  /*!
   * \brief Sets if the fields are defined.
   *
   * \param is_defined if the fields are defined.
   */
  void setIsDefined(const std::vector<bool>& is_defined);

private:
  std::string m_version_c_version;
  uint8_t m_version_major_version_number;
  uint8_t m_version_minor_version_number;
  uint8_t m_version_release_number;
  std::vector<uint32_t> m_name_length;
  std::vector<std::string> m_field_name;
  std::vector<bool> m_is_defined;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_FIELDSETS_H
