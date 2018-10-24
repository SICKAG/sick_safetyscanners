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

#ifndef FIELDDATA_H
#define FIELDDATA_H

#include <iostream>
#include <vector>

#include <sick_microscan3_ros_driver/datastructure/ScanPoint.h>



namespace sick {
namespace datastructure {


class FieldData
{
public:
  FieldData();

  int getFieldSetIndex() const;
  void setFieldSetIndex(int field_set_index);

  bool getIsWarningField() const;
  void setIsWarningField(bool is_warning_field);

  bool getIsProtectiveField() const;
  void setIsProtectiveField(bool is_protective_field);

  ScanPoint getFieldGeometry() const;
  void setFieldGeometry(const ScanPoint &field_geometry);

  std::vector<int> getBeamDistances() const;
  void setBeamDistances(const std::vector<int> &beam_distances);

private:

  int m_field_set_index;
  bool m_is_warning_field;
  bool m_is_protective_field;
  ScanPoint m_field_geometry;
  std::vector<int> m_beam_distances;

};


} // namespace datastructure
} // namespace sick

#endif
