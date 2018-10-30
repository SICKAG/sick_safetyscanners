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
 * \file Data.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_DATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_DATA_H

#include <memory>

#include <sick_safetyscanners/datastructure/ApplicationData.h>
#include <sick_safetyscanners/datastructure/ApplicationInputs.h>
#include <sick_safetyscanners/datastructure/ApplicationOutputs.h>
#include <sick_safetyscanners/datastructure/DataHeader.h>
#include <sick_safetyscanners/datastructure/DerivedValues.h>
#include <sick_safetyscanners/datastructure/GeneralSystemState.h>
#include <sick_safetyscanners/datastructure/IntrusionData.h>
#include <sick_safetyscanners/datastructure/MeasurementData.h>

namespace sick {
namespace datastructure {

/*!
 * \brief The data class containing all data blocks of a measurement.
 */
class Data
{
public:
  /*!
   * \brief Constructor of data instance.
   */
  Data();

  /*!
   * \brief Gets the data header.
   *
   * \returns The data header.
   */
  std::shared_ptr<DataHeader> getDataHeaderPtr() const;
  /*!
   * \brief Sets the data header.
   *
   * \param data_header_ptr The new data header.
   */
  void setDataHeaderPtr(const std::shared_ptr<DataHeader>& data_header_ptr);

  /*!
   * \brief Gets the general system state.
   *
   * \returns The general system state.
   */
  std::shared_ptr<GeneralSystemState> getGeneralSystemStatePtr() const;
  void
    /*!
     * \brief Sets the general system state.
     *
     * \param general_system_state_ptr The new general system state.
     */
  setGeneralSystemStatePtr(const std::shared_ptr<GeneralSystemState>& general_system_state_ptr);

  /*!
   * \brief Gets the derived values.
   *
   * \returns The derived values.
   */
  std::shared_ptr<DerivedValues> getDerivedValuesPtr() const;
  /*!
   * \brief Sets the derived values.
   *
   * \param derived_values_ptr The new derived values.
   */
  void setDerivedValuesPtr(const std::shared_ptr<DerivedValues>& derived_values_ptr);

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

  /*!
   * \brief Gets the intrusion data.
   *
   * \returns The intrusion data.
   */
  std::shared_ptr<IntrusionData> getIntrusionDataPtr() const;
  /*!
   * \brief Sets the intrusion data.
   *
   * \param intrusion_data_ptr The new intrusion data.
   */
  void setIntrusionDataPtr(const std::shared_ptr<IntrusionData>& intrusion_data_ptr);

  /*!
   * \brief Gets the application data.
   *
   * \returns The application data.
   */
  std::shared_ptr<ApplicationData> getApplicationDataPtr() const;
  /*!
   * \brief Sets the application data.
   *
   * \param application_data_ptr The new application data.
   */
  void setApplicationDataPtr(const std::shared_ptr<ApplicationData>& application_data_ptr);

private:
  std::shared_ptr<DataHeader> m_data_header_ptr;
  std::shared_ptr<GeneralSystemState> m_general_system_state_ptr;
  std::shared_ptr<DerivedValues> m_derived_values_ptr;
  std::shared_ptr<MeasurementData> m_measurement_data_ptr;
  std::shared_ptr<IntrusionData> m_intrusion_data_ptr;
  std::shared_ptr<ApplicationData> m_application_data_ptr;
};

} // namespace datastructure
} // namespace sick

#endif
