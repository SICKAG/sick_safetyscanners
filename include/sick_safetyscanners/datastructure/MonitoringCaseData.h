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
 * \file MonitoringCaseData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-11-29
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_MONITORINGCASEDATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_MONITORINGCASEDATA_H

#include <iostream>
#include <vector>

namespace sick {
namespace datastructure {


/*!
 * \brief TODO
 */
class MonitoringCaseData
{
public:
  /*!
   * \brief The constructor of the monitoring case data.
   */
  MonitoringCaseData();

  /*!
   * \brief Returns if the received monitoring case data is valid.
   *
   * \returns If the received monitoring case data is valid.
   */
  bool getIsValid() const;

  /*!
   * \brief Sets if the monitoring case data is valid.
   *
   * \param is_valid if the monitoring data is valid.
   */
  void setIsValid(bool is_valid);

private:
  bool m_is_valid;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_MONITORINGCASEDATA_H
