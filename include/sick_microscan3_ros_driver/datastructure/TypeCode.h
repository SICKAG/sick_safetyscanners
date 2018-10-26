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

#ifndef TYPECODE_H
#define TYPECODE_H

#include <iostream>


namespace sick {
namespace datastructure {

enum e_interface_type
{
  E_EFIPRO,
  E_ETHERNET_IP,
  E_PROFINET,
  E_NONSAFE_ETHERNET
};

enum e_ranges
{
  E_NORMAL_RANGE = 40,
  E_LONG_RANGE   = 64
};


class TypeCode
{
public:
  TypeCode();
  int getInterfaceType() const;
  void setInterfaceType(int interface_type);

  float getMaxRange() const;
  void setMaxRange(float max_distance);

private:
  int m_interface_type;
  float m_max_range;
};


} // namespace datastructure
} // namespace sick

#endif
