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

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_SCANPOINT_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_SCANPOINT_H

#include <stdint.h>

namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the data of a single scan point.
 */
class ScanPoint
{
public:
  ScanPoint();

  /*!
   * \brief Constructor of a scan point, takes all needed parameters.
   * \param angle Angle of the scanpoint in coordinates of the sensor.
   * \param distance Distance of the measured scanpoint.
   * \param reflectivity Value how strongly the scan point reflects.
   * \param valid_bit If the scanpoint is valid.
   * \param infinite_bit If the scanpoint is infinite.
   * \param glare_bit If there is glare in the scanpoint.
   * \param reflector_bit If the scanpoint detects a reflector.
   * \param contamination_bit If the scanpoint is contaminated.
   * \param contamination_warning_bit Warning if the scanpoint is contaminated.
   */
  ScanPoint(float angle,
            int16_t& distance,
            uint8_t& reflectivity,
            bool& valid_bit,
            bool& infinite_bit,
            bool& glare_bit,
            bool& reflector_bit,
            bool& contamination_bit,
            bool& contamination_warning_bit);

  /*!
   * \brief Getter for the angle in sensor coordinates.
   * \return The angle of the sensor scanpoint.
   */
  float getAngle() const;

  /*!
   * \brief Getter for the distance of the scanpoint.
   * \return The distance of the scanpoint.
   */
  uint16_t getDistance() const;

  /*!
   * \brief Getter for the reflectivity value.
   * \return The reflectivity value of the scanpoint.
   */
  uint8_t getReflectivity() const;

  /*!
   * \brief Returns if the scanpoint is valid.
   * \return If the scanpoint is valid.
   */
  bool getValidBit() const;

  /*!
   * \brief Returns if the scanpoint is infinite.
   * \return If the scanpoint is infinite.
   */
  bool getInfiniteBit() const;

  /*!
   * \brief Returns if the scanpoint has glare.
   * \return If the scanpoint has glare.
   */
  bool getGlareBit() const;

  /*!
   * \brief Returns if the scanpoint detects a reflector.
   * \return If the scanpoint is a reflector.
   */
  bool getReflectorBit() const;

  /*!
   * \brief Returns if the scanpoint is contaminated.
   * \return If the scanpoint is contaminated.
   */
  bool getContaminationBit() const;

  /*!
   * \brief Returns if there is a contamination warning.
   * \return If there is a contamination warning.
   */
  bool getContaminationWarningBit() const;


private:
  float m_angle;
  int16_t m_distance; // in mm
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

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_SCANPOINT_H
