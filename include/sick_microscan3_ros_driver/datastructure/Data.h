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

#ifndef DATA_H
#define DATA_H

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

#include <sick_microscan3_ros_driver/datastructure/ApplicationData.h>
#include <sick_microscan3_ros_driver/datastructure/ApplicationInputs.h>
#include <sick_microscan3_ros_driver/datastructure/ApplicationOutputs.h>
#include <sick_microscan3_ros_driver/datastructure/DataHeader.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>
#include <sick_microscan3_ros_driver/datastructure/GeneralSystemState.h>
#include <sick_microscan3_ros_driver/datastructure/IntrusionData.h>
#include <sick_microscan3_ros_driver/datastructure/MeasurementData.h>

namespace sick {
namespace datastructure {

class Data
{
public:
  Data();

  boost::shared_ptr<DataHeader> getDataHeaderPtr() const;
  void setDataHeaderPtr(const boost::shared_ptr<DataHeader>& data_header_ptr);

  boost::shared_ptr<GeneralSystemState> getGeneralSystemStatePtr() const;
  void
  setGeneralSystemStatePtr(const boost::shared_ptr<GeneralSystemState>& general_system_state_ptr);

  boost::shared_ptr<DerivedValues> getDerivedValuesPtr() const;
  void setDerivedValuesPtr(const boost::shared_ptr<DerivedValues>& derived_values_ptr);

  boost::shared_ptr<MeasurementData> getMeasurementDataPtr() const;
  void setMeasurementDataPtr(const boost::shared_ptr<MeasurementData>& measurement_data_ptr);

  boost::shared_ptr<IntrusionData> getIntrusionDataPtr() const;
  void setIntrusionDataPtr(const boost::shared_ptr<IntrusionData>& intrusion_data_ptr);

  boost::shared_ptr<ApplicationData> getApplicationDataPtr() const;
  void setApplicationDataPtr(const boost::shared_ptr<ApplicationData>& application_data_ptr);

private:
  boost::shared_ptr<DataHeader> m_data_header_ptr;
  boost::shared_ptr<GeneralSystemState> m_general_system_state_ptr;
  boost::shared_ptr<DerivedValues> m_derived_values_ptr;
  boost::shared_ptr<MeasurementData> m_measurement_data_ptr;
  boost::shared_ptr<IntrusionData> m_intrusion_data_ptr;
  boost::shared_ptr<ApplicationData> m_application_data_ptr;
};

} // namespace datastructure
} // namespace sick

#endif
