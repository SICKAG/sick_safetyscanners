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
 * \file ApplicationOutputs.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_APPLICATIONOUTPUTS_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_APPLICATIONOUTPUTS_H

#include <stdint.h>
#include <vector>


namespace sick {
namespace datastructure {

class ApplicationOutputs
{
public:
  ApplicationOutputs();

  std::vector<bool> getEvalOutVector() const;
  void setEvalOutVector(const std::vector<bool>& eval_out_vector);

  std::vector<bool> getEvalOutIsSafeVector() const;
  void setEvalOutIsSafeVector(const std::vector<bool>& eval_out_is_safe_vector);

  std::vector<bool> getEvalOutIsValidVector() const;
  void setEvalOutIsValidVector(const std::vector<bool>& eval_out_is_valid_vector);

  std::vector<uint16_t> getMonitoringCaseVector() const;
  void setMonitoringCaseVector(const std::vector<uint16_t>& monitoring_case_vector);

  std::vector<bool> getMonitoringCaseFlagsVector() const;
  void setMonitoringCaseFlagsVector(const std::vector<bool>& monitoring_case_flags_vector);

  int8_t getSleepModeOutput() const;
  void setSleepModeOutput(const int8_t& sleep_mode_output);

  bool getHostErrorFlagContaminationWarning() const;
  void setHostErrorFlagContaminationWarning(bool host_error_flag_contamination_warning);

  bool getHostErrorFlagContaminationError() const;
  void setHostErrorFlagContaminationError(bool host_error_flag_contamination_error);

  bool getHostErrorFlagManipulationError() const;
  void setHostErrorFlagManipulationError(bool host_error_flag_manipulation_error);

  bool getHostErrorFlagGlare() const;
  void setHostErrorFlagGlare(bool host_error_flag_glare);

  bool getHostErrorFlagReferenceContourIntruded() const;
  void setHostErrorFlagReferenceContourIntruded(bool host_error_flag_reference_contour_intruded);

  bool getHostErrorFlagCriticalError() const;
  void setHostErrorFlagCriticalError(bool host_error_flag_critical_error);

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

  std::vector<int16_t> getResultingVelocityVector() const;
  void setResultingVelocityVector(const std::vector<int16_t>& resulting_velocity_vector);

  std::vector<bool> getResultingVelocityIsValidVector() const;
  void
  setResultingVelocityIsValidVector(const std::vector<bool>& resulting_velocity_is_valid_vector);

  bool getFlagsSleepModeOutputIsValid() const;
  void setFlagsSleepModeOutputIsValid(bool flags_sleep_mode_output_is_valid);

  bool getFlagsHostErrorFlagsAreValid() const;
  void setFlagsHostErrorFlagsAreValid(bool flags_host_error_flags_are_valid);

private:
  std::vector<bool> m_eval_out_vector;
  std::vector<bool> m_eval_out_is_safe_vector;
  std::vector<bool> m_eval_out_is_valid_vector;

  std::vector<uint16_t> m_monitoring_case_vector;
  std::vector<bool> m_monitoring_case_flags_vector;

  int8_t m_sleep_mode_output;

  bool m_host_error_flag_contamination_warning;
  bool m_host_error_flag_contamination_error;
  bool m_host_error_flag_manipulation_error;
  bool m_host_error_flag_glare;
  bool m_host_error_flag_reference_contour_intruded;
  bool m_host_error_flag_critical_error;

  int16_t m_velocity_0;
  int16_t m_velocity_1;

  bool m_velocity_0_valid;
  bool m_velocity_1_valid;
  bool m_velocity_0_transmitted_safely;
  bool m_velocity_1_transmitted_safely;

  std::vector<int16_t> m_resulting_velocity_vector;
  std::vector<bool> m_resulting_velocity_is_valid_vector;

  bool m_flags_sleep_mode_output_is_valid;
  bool m_flags_host_error_flags_are_valid;
};


}  // namespace datastructure
}  // namespace sick

#endif
