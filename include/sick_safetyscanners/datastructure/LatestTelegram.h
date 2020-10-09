// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2019, SICK AG, Waldkirch
*  Copyright (C) 2019, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file LatestTelegram.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_LATESTTELEGRAM_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_LATESTTELEGRAM_H

#include <iostream>
#include <memory>
#include <sick_safetyscanners/datastructure/MeasurementData.h>


namespace sick {
namespace datastructure {

/*!
 * \brief Class containing the latest telegram of a laser scanner.
 */
class LatestTelegram
{
public:
  /*!
   * \brief Constructor of the latest telegram.
   */
  LatestTelegram();

  /*!
   * \brief Gets the measurement data.
   *
   * \returns The measurement data.
   */
  std::shared_ptr<MeasurementData> getMeasurementDataPtr() const;
  /*!
   * \brief Sets the measurement data.
   *
   * \param measurement_data_ptr The new measurement data.
   */
  void setMeasurementDataPtr(const std::shared_ptr<MeasurementData>& measurement_data_ptr);

private:
  std::shared_ptr<MeasurementData> m_measurement_data_ptr;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_LATESTTELEGRAM_H
