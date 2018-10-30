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
 * \file MeasurementData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_MEASUREMENTDATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_MEASUREMENTDATA_H

#include <stdint.h>
#include <vector>

#include <sick_safetyscanners/datastructure/ScanPoint.h>

namespace sick {
namespace datastructure {

/*!
 * \brief Class containing all scanpoints of a single measurement.
 */
class MeasurementData
{
public:
  /*!
   * \brief Constructor of an empty measurement.
   */
  MeasurementData();

  /*!
   * \brief Getter for the number of beams.
   * \return Return number of beams.
   */
  uint32_t getNumberOfBeams() const;

  /*!
   * \brief Setter for the number of beams.
   * \param number_of_beams Input number of beams.
   */
  void setNumberOfBeams(const uint32_t& number_of_beams);

  /*!
   * \brief Getter for all contained scanpoints.
   * \return Vector of scanpoints.
   */
  std::vector<ScanPoint> getScanPointsVector() const;

  /*!
   * \brief Add a single scanpoint to the vector of scanpoints.
   * \param scan_point New scanpoint to add.
   */
  void addScanPoint(ScanPoint scan_point);

  /*!
   * \brief Returns if measurement data has been enabled.
   * \return If measurement data has been enabled.
   */
  bool isEmpty() const;

  /*!
   * \brief Set if measurement data is enabled
   * \param is_empty set if measurement data is enabled.
   */
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;

  uint32_t m_number_of_beams;
  std::vector<ScanPoint> m_scan_points_vector;
};

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_MEASUREMENTDATA_H
