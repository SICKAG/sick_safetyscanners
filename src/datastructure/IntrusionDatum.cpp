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
 * \file IntrusionDatum.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/IntrusionDatum.h>

namespace sick {
namespace datastructure {


IntrusionDatum::IntrusionDatum() {}

int32_t IntrusionDatum::getSize() const
{
  return m_size;
}

void IntrusionDatum::setSize(const int32_t& size)
{
  m_size = size;
}

std::vector<bool> IntrusionDatum::getFlagsVector() const
{
  return m_flags_vector;
}

void IntrusionDatum::setFlagsVector(const std::vector<bool>& flags_vector)
{
  m_flags_vector = flags_vector;
}

} // namespace datastructure
} // namespace sick
