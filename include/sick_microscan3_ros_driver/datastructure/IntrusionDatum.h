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
 * \file IntrusionDatum.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class IntrusionDatum
{
public:
  IntrusionDatum();
  INT32 getSize() const;
  void setSize(const INT32& size);

  std::vector<bool> getFlagsVector() const;
  void setFlagsVector(const std::vector<bool>& flags_vector);

private:
  INT32 m_size;
  std::vector<bool> m_flags_vector;
};

} // namespace datastructure
} // namespace sick
