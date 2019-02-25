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
 * \file ConfigData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-02-25
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGDATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGDATA_H

#include <iostream>
#include <vector>

namespace sick {
namespace datastructure {


/*!
 * \brief Config data for current and persistent sensor config.
 */
class ConfigData
{
public:
  /*!
   * \brief The constructor of the config data.
   */
  ConfigData();


  /*!
   * \brief Get the start angle of the configuration.
   * \return Start angle of the configuration.
   */
  float getStartAngle() const;

  /*!
   * \brief Set the start angle of the configuration.
   * \param start_angle Start angle of the scan.
   */
  void setStartAngle(const int32_t& start_angle);

  /*!
   * \brief Set the start angle of the configuration from degrees.
   * \param start_angle Start angle of the configuration in degrees.
   */
  void setStartAngleDegrees(const float& start_angle);

  /*!
   * \brief Get the end angle of the configuration.
   * \return End angle of the configuration.
   */
  float getEndAngle() const;

  /*!
   * \brief Set the end angle of the configuration.
   * \param end_angle End angle of the configuration.
   */
  void setEndAngle(const int32_t& end_angle);

  /*!
   * \brief Set the end angle of the configuration from degrees.
   * \param end_angle End angle of the configuration in degrees.
   */
  void setEndAngleDegrees(const float& end_angle);

  /*!
   * \brief Returns the angular resolution between the beams.
   * \return Angular resolution between beams.
   */
  float getAngularBeamResolution() const;

  /*!
   * \brief Set the angular resolution between beams.
   * \param angular_beam_resolution The angular resolution between two beams.
   */
  void setAngularBeamResolution(const int32_t& angular_beam_resolution);

  /*!
   * \brief Set the angular resolution between beams from degrees.
   * \param angular_beam_resolution The angular resolution between two beams in degrees.
   */
  void setAngularBeamResolutionDegrees(const float& angular_beam_resolution);

private:
  /*!
   * \brief Defined angle resolution to convert sensor input to the right frame
   */
  const double ANGLE_RESOLUTION = 4194304.0;

  float m_start_angle;
  float m_end_angle;
  float m_angular_beam_resolution;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_CONFIGDATA_H
