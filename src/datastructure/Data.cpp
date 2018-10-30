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
 * \file Data.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/Data.h>

namespace sick {
namespace datastructure {

Data::Data() {}

std::shared_ptr<DataHeader> Data::getDataHeaderPtr() const
{
  return m_data_header_ptr;
}


void Data::setDataHeaderPtr(const std::shared_ptr<DataHeader>& data_header_ptr)
{
  m_data_header_ptr = data_header_ptr;
}

std::shared_ptr<GeneralSystemState> Data::getGeneralSystemStatePtr() const
{
  return m_general_system_state_ptr;
}

void Data::setGeneralSystemStatePtr(
  const std::shared_ptr<GeneralSystemState>& general_system_state_ptr)
{
  m_general_system_state_ptr = general_system_state_ptr;
}

std::shared_ptr<DerivedValues> Data::getDerivedValuesPtr() const
{
  return m_derived_values_ptr;
}

void Data::setDerivedValuesPtr(const std::shared_ptr<DerivedValues>& derived_values_ptr)
{
  m_derived_values_ptr = derived_values_ptr;
}

std::shared_ptr<MeasurementData> Data::getMeasurementDataPtr() const
{
  return m_measurement_data_ptr;
}

void Data::setMeasurementDataPtr(const std::shared_ptr<MeasurementData>& measurement_data_ptr)
{
  m_measurement_data_ptr = measurement_data_ptr;
}

std::shared_ptr<IntrusionData> Data::getIntrusionDataPtr() const
{
  return m_intrusion_data_ptr;
}

void Data::setIntrusionDataPtr(const std::shared_ptr<IntrusionData>& intrusion_data_ptr)
{
  m_intrusion_data_ptr = intrusion_data_ptr;
}

std::shared_ptr<ApplicationData> Data::getApplicationDataPtr() const
{
  return m_application_data_ptr;
}

void Data::setApplicationDataPtr(const std::shared_ptr<ApplicationData>& application_data_ptr)
{
  m_application_data_ptr = application_data_ptr;
}


} // namespace datastructure
} // namespace sick
