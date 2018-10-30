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

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_INTRUSIONDATUM_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_INTRUSIONDATUM_H

#include <stdint.h>
#include <vector>

namespace sick {
namespace datastructure {

/*!
 * \brief Class containing a single IntrusionDatum.
 */
class IntrusionDatum
{
public:
  /*!
   * \brief Constructor of an empty IntrusionDatum.
   */
  IntrusionDatum();

  /*!
   * \brief Returns size of flag vector.
   * \return Size of flag vector.
   */
  int32_t getSize() const;

  /*!
   * \brief Setter of the size for the flag vector.
   * \param size Size of the flag vector.
   */
  void setSize(const int32_t& size);

  /*!
   * \brief Getter for the flags vector.
   * \return Boolean vector of all flags
   */
  std::vector<bool> getFlagsVector() const;

  /*!
   * \brief Setter for the flag vector
   * \param flags_vector Bool vector for all flags.
   */
  void setFlagsVector(const std::vector<bool>& flags_vector);

private:
  int32_t m_size;
  std::vector<bool> m_flags_vector;
};

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_INTRUSIONDATUM_H
