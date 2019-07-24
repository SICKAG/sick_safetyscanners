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
 * \file TypeCode.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/TypeCode.h>

namespace sick {
namespace datastructure {

TypeCode::TypeCode() {}

std::string TypeCode::getTypeCode() const
{
  return m_type_code;
}

void TypeCode::setTypeCode(const std::string& type_code)
{
  m_type_code = type_code;
}
uint8_t TypeCode::getInterfaceType() const
{
  return m_interface_type;
}

void TypeCode::setInterfaceType(const uint8_t& interface_type)
{
  m_interface_type = interface_type;
}

float TypeCode::getMaxRange() const
{
  return m_max_range;
}

void TypeCode::setMaxRange(const float& max_distance)
{
  m_max_range = max_distance;
}


} // namespace datastructure
} // namespace sick
