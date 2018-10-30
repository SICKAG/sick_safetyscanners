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
 * \file IntrusionData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/IntrusionData.h>

namespace sick {
namespace datastructure {

IntrusionData::IntrusionData()
  : m_is_empty(false)
{
}

std::vector<IntrusionDatum> IntrusionData::getIntrusionDataVector() const
{
  return m_intrusion_data_vector;
}

void IntrusionData::setIntrusionDataVector(const std::vector<IntrusionDatum>& intrusion_data_vector)
{
  m_intrusion_data_vector = intrusion_data_vector;
}

bool IntrusionData::isEmpty() const
{
  return m_is_empty;
}

void IntrusionData::setIsEmpty(bool is_empty)
{
  m_is_empty = is_empty;
}


} // namespace datastructure
} // namespace sick
