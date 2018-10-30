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
 * \file IntrusionData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_INTRUSIONDATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_INTRUSIONDATA_H

#include <sick_safetyscanners/datastructure/IntrusionDatum.h>

#include <vector>

namespace sick {
namespace datastructure {

/*!
 * \brief Class containing all IntrusionDatums.
 */
class IntrusionData
{
public:
  /*!
   * \brief Constructor for an empty IntrusionData Object.
   */
  IntrusionData();

  /*!
   * \brief Getter for all IntrusionDatums.
   * \return Vector of IntrusionDatum.
   */
  std::vector<IntrusionDatum> getIntrusionDataVector() const;

  /*!
   * \brief Setter for the vector of IntrusionDatums.
   * \param intrusion_data_vector Vector of IntrusionDatums.
   */
  void setIntrusionDataVector(const std::vector<IntrusionDatum>& intrusion_data_vector);

  /*!
   * \brief Return if intrusion data has been enabled.
   * \return If intrusion data has been enabled.
   */
  bool isEmpty() const;

  /*!
   * \brief Set if intrusion data has been enabled.
   * \param is_empty If intrusion data has been enabled.
   */
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;
  std::vector<IntrusionDatum> m_intrusion_data_vector;
};

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_INTRUSIONDATA_H
