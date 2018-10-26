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
 * \file ApplicationInputs.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_APPLICATIONINPUTS_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_APPLICATIONINPUTS_H

#include <stdint.h>
#include <vector>

namespace sick {
namespace datastructure {

class ApplicationInputs
{
public:
  ApplicationInputs();

  std::vector<bool> getUnsafeInputsInputSourcesVector() const;
  void
  setUnsafeInputsInputSourcesVector(const std::vector<bool>& unsafe_inputs_input_sources_vector);

  std::vector<bool> getUnsafeInputsFlagsVector() const;
  void setUnsafeInputsFlagsVector(const std::vector<bool>& unsafe_inputs_flags_vector);

  std::vector<uint16_t> getMonitoringCasevector() const;
  void setMonitoringCaseVector(const std::vector<uint16_t>& monitoring_case_vector);

  std::vector<bool> getMonitoringCaseFlagsVector() const;
  void setMonitoringCaseFlagsVector(const std::vector<bool>& monitoring_case_flags_vector);

  int16_t getVelocity0() const;
  void setVelocity0(const int16_t& velocity_0);

  int16_t getVelocity1() const;
  void setVelocity1(const int16_t& velocity_1);

  bool getVelocity0Valid() const;
  void setVelocity0Valid(bool velocity_0_valid);

  bool getVelocity1Valid() const;
  void setVelocity1Valid(bool velocity_1_valid);

  bool getVelocity0TransmittedSafely() const;
  void setVelocity0TransmittedSafely(bool velocity_0_transmitted_safely);

  bool getVelocity1TransmittedSafely() const;
  void setVelocity1TransmittedSafely(bool velocity_1_transmitted_safely);

  int8_t getSleepModeInput() const;
  void setSleepModeInput(const int8_t& sleep_mode_input);

private:
  std::vector<bool> m_unsafe_inputs_input_sources_vector;
  std::vector<bool> m_unsafe_inputs_flags_vector;

  std::vector<uint16_t> m_monitoring_case_vector;
  std::vector<bool> m_monitoring_case_flags_vector;

  int16_t m_velocity_0;
  int16_t m_velocity_1;

  bool m_velocity_0_valid;
  bool m_velocity_1_valid;
  bool m_velocity_0_transmitted_safely;
  bool m_velocity_1_transmitted_safely;

  int8_t m_sleep_mode_input;
};

}  // namespace datastructure
}  // namespace sick

#endif
