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

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_FIELDDATA_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_FIELDDATA_H

#include <iostream>
#include <vector>

#include <sick_microscan3_ros_driver/datastructure/ScanPoint.h>


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
   * \brief Returns the field geometry data as scanpoint.
   *
   * Not implemented yet.
   *
   * \returns The field geometry data as a scanpoint.
   */
  ScanPoint getFieldGeometry() const;

  /*!
   * \brief Sets the field geometry as scanpoints.
   *
   * \param field_geometry The new scanpoint.
   */
  void setFieldGeometry(const ScanPoint& field_geometry);

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

private:
  uint16_t m_field_set_index;
  bool m_is_warning_field;
  bool m_is_protective_field;
  ScanPoint m_field_geometry;
  std::vector<uint16_t> m_beam_distances;  // in mm
};


} // namespace datastructure
} // namespace sick

#endif // SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_FIELDDATA_H
