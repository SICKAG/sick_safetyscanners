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
 * \file FieldData.cpp
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
  void setFieldSetIndex(uint16_t& field_set_index);

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
  const double ANGLE_RESOLUTION = 4194304.0;

  uint16_t m_field_set_index;
  bool m_is_warning_field;
  bool m_is_protective_field;
  std::vector<uint16_t> m_beam_distances; // in mm
  float m_start_angle;
  float m_angular_beam_resolution;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_FIELDDATA_H
