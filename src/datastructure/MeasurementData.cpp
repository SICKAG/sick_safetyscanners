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
 * \file MeasurementData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/MeasurementData.h>

namespace sick {
namespace datastructure {


MeasurementData::MeasurementData()
  : m_is_empty(false)
{
}

uint32_t MeasurementData::getNumberOfBeams() const
{
  return m_number_of_beams;
}

void MeasurementData::setNumberOfBeams(const uint32_t& number_of_beams)
{
  m_number_of_beams = number_of_beams;
}

std::vector<ScanPoint> MeasurementData::getScanPointsVector() const
{
  return m_scan_points_vector;
}

void MeasurementData::addScanPoint(ScanPoint scan_point)
{
  m_scan_points_vector.push_back(scan_point);
}

bool MeasurementData::isEmpty() const
{
  return m_is_empty;
}

void MeasurementData::setIsEmpty(bool is_empty)
{
  m_is_empty = is_empty;
}


} // namespace datastructure
} // namespace sick
