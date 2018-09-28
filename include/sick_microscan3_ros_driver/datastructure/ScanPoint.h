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
 * \file ScanPoint.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SCANPOINT_H
#define SCANPOINT_H

#include <stdint.h>

namespace sick {
namespace datastructure {

class ScanPoint
{
public:
  ScanPoint();
  ScanPoint(float angle,
            int16_t& distance,
            uint8_t& reflectivity,
            bool& valid_bit,
            bool& infinite_bit,
            bool& glare_bit,
            bool& reflector_bit,
            bool& contamination_bit,
            bool& contamination_warning_bit);

  float getAngle() const;

  uint16_t getDistance() const;

  uint8_t getReflectivity() const;

  bool getValidBit() const;

  bool getInfiniteBit() const;

  bool getGlareBit() const;

  bool getReflectorBit() const;

  bool getContaminationBit() const;

  bool getContaminationWarningBit() const;


private:
  float m_angle;
  int16_t m_distance;
  uint8_t m_reflectivity;
  bool m_valid_bit;
  bool m_infinite_bit;
  bool m_glare_bit;
  bool m_reflector_bit;
  bool m_contamination_bit;
  bool m_contamination_warning_bit;
};


} // namespace datastructure
} // namespace sick

#endif
