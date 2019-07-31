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
 * \file FieldData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_FIELDDATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_FIELDDATA_H

#include <iostream>
#include <vector>

namespace sick {
namespace datastructure {


/*!
 * \brief Field data for warning and protective fields.
 */
class FieldData
{
public:
  /*!
   * \brief The constructor of the field data.
   */
  FieldData();

  /*!
   * \brief Returns if the received field data is valid.
   *
   * \returns If the received field data is valid.
   */
  bool getIsValid() const;

  /*!
   * \brief Sets if the field data is valid.
   *
   * \param is_valid if the field data is valid.
   */
  void setIsValid(bool is_valid);
  ;
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
   * \brief Returns if the received field data is defined.
   *
   * \returns If the received field data is defined.
   */
  bool getIsDefined() const;

  /*!
   * \brief Sets if the field data is defined.
   *
   * \param is_defined if the field data is defined.
   */
  void setIsDefined(bool is_defined);
  ;

  /*!
   * \brief Returns the configured eval method.
   *
   * \returns The eval method.
   */
  uint8_t getEvalMethod() const;

  /*!
   * \brief Sets the configured eval method.
   *
   * \param eval_method The configured eval method.
   */
  void setEvalMethod(const uint8_t& eval_method);

  /*!
   * \brief Returns the multiple sampling of the field.
   *
   * \returns The configured multiple sampling for this field.
   */
  uint16_t getMultiSampling() const;

  /*!
   * \brief Sets the configured multiple sampling of the field.
   *
   * \param multi_sampling The configured multiple sampling.
   */
  void setMultiSampling(const uint16_t& multi_sampling);
  ;

  /*!
   * \brief Returns the configured object resolution.
   *
   * \returns The configured object resolution.
   */
  uint16_t getObjectResolution() const;

  /*!
   * \brief Sets the configured object resolution.
   *
   * \param object_resolution The configured object resolution.
   */
  void setObjectResolution(const uint16_t& object_resolution);
  ;
  /*!
   * \brief Returns the index of the field set the field belongs to.
   *
   * \returns The index of the field set the field belongs to.
   */
  uint16_t getFieldSetIndex() const;

  /*!
   * \brief Sets the index of the field set where the field belongs to.
   *
   * \param field_set_index The index of the field set where the field belongs to.
   */
  void setFieldSetIndex(const uint16_t& field_set_index);
  /*!
   * \brief Gets the length of the field name.
   *
   * \returns The length of the field name.
   */
  uint32_t getNameLength() const;
  /*!
   * \brief Sets the length of the field name.
   *
   * \param name_length The length of the field name.
   */
  void setNameLength(const uint32_t& name_length);
  /*!
   * \brief Gets the current field name.
   *
   * \returns The current field name.
   */
  std::string getFieldName() const;
  /*!
   * \brief Sets the field name.
   *
   * \param field_name The field name.
   */
  void setFieldName(const std::string& field_name);

  /*!
   * \brief Returns if a field is warning field.
   *
   * \returns If field is a warning field.
   */
  bool getIsWarningField() const;

  /*!
   * \brief Set if a field is a warning field.
   *
   * \param is_warning_field Set if a field is a warning field.
   */
  void setIsWarningField(bool is_warning_field);

  /*!
   * \brief Returns if a field is a protective field.
   *
   * \returns If a field is protective.
   */
  bool getIsProtectiveField() const;

  /*!
   * \brief Set if a field is protective field.
   *
   * \param is_protective_field Set if a field is a protective field.
   */
  void setIsProtectiveField(bool is_protective_field);

  /*!
   * \brief Returns vector with beam distances.
   *
   * \returns Vector with beam distances.
   */
  std::vector<uint16_t> getBeamDistances() const;

  /*!
   * \brief Sets vector with beam distances for field.
   *
   * \param beam_distances New beam distances for field.
   */
  void setBeamDistances(const std::vector<uint16_t>& beam_distances);

  /*!
   * \brief Get the start angle of the scan.
   * \return Start angle of the scan.
   */
  float getStartAngle() const;

  /*!
   * \brief Set the start angle of the scan.
   * \param start_angle Start angle of the scan.
   */
  void setStartAngle(const int32_t& start_angle);

  /*!
   * \brief Set the start angle of the scan from degrees.
   * \param start_angle Start angle of the scan in degrees.
   */
  void setStartAngleDegrees(const float& start_angle);

  /*!
   * \brief Get the end angle of the scan.
   * \return End angle of the scan.
   */
  float getEndAngle() const;

  /*!
   * \brief Set the end angle of the scan.
   * \param end_angle End angle of the scan.
   */
  void setEndAngle(const int32_t& end_angle);

  /*!
   * \brief Set the end angle of the scan from degrees.
   * \param end_angle End angle of the scan in degrees.
   */
  void setEndAngleDegrees(const float& end_angle);

  /*!
   * \brief Returns the angular resolution between the beams.
   * \return Angular resolution between beams.
   */
  float getAngularBeamResolution() const;

  /*!
   * \brief Set the angular resolution between beams.
   * \param angular_beam_resolution The angular resolution between two beams.
   */
  void setAngularBeamResolution(const int32_t& angular_beam_resolution);

  /*!
   * \brief Set the angular resolution between beams from degrees.
   * \param angular_beam_resolution The angular resolution between two beams in degrees.
   */
  void setAngularBeamResolutionDegrees(const float& angular_beam_resolution);

private:
  /*!
   * \brief Defined angle resolution to convert sensor input to the right frame
   */
  const double m_ANGLE_RESOLUTION = 4194304.0;

  std::string m_version_c_version;
  uint8_t m_version_major_version_number;
  uint8_t m_version_minor_version_number;
  uint8_t m_version_release_number;
  bool m_is_valid;
  bool m_is_defined;
  uint8_t m_eval_method;
  uint16_t m_multi_sampling;
  uint16_t m_object_resolution;
  uint16_t m_field_set_index;
  uint32_t m_name_length;
  std::string m_field_name;
  bool m_is_warning_field;
  bool m_is_protective_field;
  std::vector<uint16_t> m_beam_distances; // in mm
  float m_start_angle;
  float m_end_angle;
  float m_angular_beam_resolution;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_FIELDDATA_H
