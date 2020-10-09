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

#include <sick_safetyscanners/datastructure/MonitoringCaseData.h>

namespace sick {
namespace datastructure {

MonitoringCaseData::MonitoringCaseData() {}

bool MonitoringCaseData::getIsValid() const
{
  return m_is_valid;
}

void MonitoringCaseData::setIsValid(bool is_valid)
{
  m_is_valid = is_valid;
}

uint16_t MonitoringCaseData::getMonitoringCaseNumber() const
{
  return m_monitoring_case_number;
}

void MonitoringCaseData::setMonitoringCaseNumber(const uint16_t& monitoring_case_number)
{
  m_monitoring_case_number = monitoring_case_number;
}

std::vector<uint16_t> MonitoringCaseData::getFieldIndices() const
{
  return m_field_indices;
}

void MonitoringCaseData::setFieldIndices(const std::vector<uint16_t>& field_indices)
{
  m_field_indices = field_indices;
}

std::vector<bool> MonitoringCaseData::getFieldsValid() const
{
  return m_fields_valid;
}

void MonitoringCaseData::setFieldsValid(const std::vector<bool>& fields_valid)
{
  m_fields_valid = fields_valid;
}

} // namespace datastructure
} // namespace sick
