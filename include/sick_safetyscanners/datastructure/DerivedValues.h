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
 * \file DerivedValues.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_DERIVEDVALUES_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_DERIVEDVALUES_H

#include <stdint.h>

namespace sick {
namespace datastructure {


/*!
 * \brief The DerivedValues class
 * Includes the derived configuration of the measurement data channel.
 */
class DerivedValues
{
public:
  /*!
   * \brief Constructor of an empty DerivedValues instance
   */
  DerivedValues();

  /*!
   * \brief Return the multiplication factor.
   *  Multiplication factor to be applied to the beam distance values to
   *  get the distance in millimeter.
   * \return The multiplication factor.
   */
  uint16_t getMultiplicationFactor() const;

  /*!
   * \brief Sets the multiplication factor.
   * \param multiplication_factor The new multiplication factor.
   */
  void setMultiplicationFactor(const uint16_t& multiplication_factor);

  /*!
   * \brief Returns the number of beams of the current scan.
   * \return Number of beams.
   */
  uint16_t getNumberOfBeams() const;

  /*!
   * \brief Sets the number of beams for the current scan
   * \param number_of_beams Number of beams for the scan.
   */
  void setNumberOfBeams(const uint16_t& number_of_beams);

  /*!
   * \brief Return the time of the scan.
   * \return  Time of the scan.
   */
  uint16_t getScanTime() const;

  /*!
   * \brief Sets the time of the scan
   * \param scan_time Time of the scan.
   */
  void setScanTime(const uint16_t& scan_time);

  /*!
   * \brief Get the start angle of the scan.
   * \return Start angle of the scan.
   */
  float getStartAngle() const;

  /*!
   * \brief Set the start angle of the scan.
   * \param start_angle Start angle of the scan.
   */
  void setStartAngle(const int32_t& start_angle);

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
   * \brief Return the time between consecutive beams.
   * \return  Time between consecutive beams.
   */
  uint32_t getInterbeamPeriod() const;

  /*!
   * \brief Set the time between two consecutive beams.
   * \param interbeam_period Time between two consecutive beams.
   */
  void setInterbeamPeriod(const uint32_t& interbeam_period);

  /*!
   * \brief Returns if derived values have been enabled.
   * \return If derived values have been enabled.
   */
  bool isEmpty() const;

  /*!
   * \brief Set if derived values are enabled
   * \param is_empty set if derived values are enabled.
   */
  void setIsEmpty(bool is_empty);

private:
  /*!
   * \brief Defined angle resolution to convert sensor input to the right frame
   */
  const double m_ANGLE_RESOLUTION = 4194304.0;

  bool m_is_empty;

  uint16_t m_multiplication_factor;
  uint16_t m_number_of_beams;
  uint16_t m_scan_time; // ms
  float m_start_angle;
  float m_angular_beam_resolution;
  uint32_t m_interbeam_period; // usecs
};

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_DERIVEDVALUES_H
