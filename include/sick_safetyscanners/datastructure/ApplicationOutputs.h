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

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONOUTPUTS_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONOUTPUTS_H

#include <stdint.h>
#include <vector>


namespace sick {
namespace datastructure {

/*!
 * \brief The application outputs from a udp data packet.
 */
class ApplicationOutputs
{
public:
  /*!
   * \brief Constructor of the application outputs.
   */
  ApplicationOutputs();

  /*!
   * \brief Gets the state of the non safe cut-off paths.
   *
   * \returns The state of the non safe cut-off paths.
   */
  std::vector<bool> getEvalOutVector() const;
  /*!
   * \brief Set the state of the non safe cut-off paths.
   *
   * \param eval_out_vector The state of the non safe cut-off paths.
   */
  void setEvalOutVector(const std::vector<bool>& eval_out_vector);

  /*!
   * \brief Gets if a cut-off path from the output paths is safe.
   *
   * \returns If a cut-off path is safe.
   */
  std::vector<bool> getEvalOutIsSafeVector() const;
  /*!
   * \brief Sets if an cut-off path is safe.
   *
   * \param eval_out_is_safe_vector If a cut off path is safe.
   */
  void setEvalOutIsSafeVector(const std::vector<bool>& eval_out_is_safe_vector);

  /*!
   * \brief If the output path is valid.
   *
   * \returns If the output path is valid.
   */
  std::vector<bool> getEvalOutIsValidVector() const;
  /*!
   * \brief Sets if an output path is valid.
   *
   * \param eval_out_is_valid_vector If an output path is valid.
   */
  void setEvalOutIsValidVector(const std::vector<bool>& eval_out_is_valid_vector);

  /*!
   * \brief Gets the currently active monitoring case numbers.
   *
   * \returns The monitoring case numbers.
   */
  std::vector<uint16_t> getMonitoringCaseVector() const;
  /*!
   * \brief Sets the currently active monitoring case numbers.
   *
   * \param monitoring_case_vector The monitoring case numbers.
   */
  void setMonitoringCaseVector(const std::vector<uint16_t>& monitoring_case_vector);

  /*!
   * \brief Gets if the corresponding monitoring case number is valid.
   *
   * \returns If the monitoring case number is valid.
   */
  std::vector<bool> getMonitoringCaseFlagsVector() const;
  /*!
   * \brief Set is the corresponding monitoring case number is valid.
   *
   * \param monitoring_case_flags_vector If the monitoring case number is valid.
   */
  void setMonitoringCaseFlagsVector(const std::vector<bool>& monitoring_case_flags_vector);

  /*!
   * \brief Gets the state of the sleep mode.
   *
   * \returns The state of the sleep mode.
   */
  int8_t getSleepModeOutput() const;
  /*!
   * \brief Sets the state of the sleep mode.
   *
   * \param sleep_mode_output The state of the sleep mode.
   */
  void setSleepModeOutput(const int8_t& sleep_mode_output);

  /*!
   * \brief Gets if a contamination warning is present.
   *
   * \returns if a contamination warning is present.
   */
  bool getHostErrorFlagContaminationWarning() const;
  /*!
   * \brief  Sets if a contamination warning is present.
   *
   * \param host_error_flag_contamination_warning If a contamination warning is present.
   */
  void setHostErrorFlagContaminationWarning(bool host_error_flag_contamination_warning);

  /*!
   * \brief Gets if a contamination error is present.
   *
   * \returns If a contamination error is present.
   */
  bool getHostErrorFlagContaminationError() const;
  /*!
   * \brief Sets if a contamination error is present.
   *
   * \param host_error_flag_contamination_error If a contamination error is present.
   */
  void setHostErrorFlagContaminationError(bool host_error_flag_contamination_error);

  /*!
   * \brief Gets if a manipulation error is present.
   *
   * \returns if a manipulation error is present.
   */
  bool getHostErrorFlagManipulationError() const;
  /*!
   * \brief Sets if a manipulation error is present.
   *
   * \param host_error_flag_manipulation_error If a manipulation error is present.
   */
  void setHostErrorFlagManipulationError(bool host_error_flag_manipulation_error);

  /*!
   * \brief Gets if glare is present.
   *
   * \returns If glare is present.
   */
  bool getHostErrorFlagGlare() const;
  /*!
   * \brief Sets if glare is present.
   *
   * \param host_error_flag_glare If glare is present.
   */
  void setHostErrorFlagGlare(bool host_error_flag_glare);

  /*!
   * \brief Gets if a reference contour is intruded.
   *
   * \returns If a reference contour is intruded.
   */
  bool getHostErrorFlagReferenceContourIntruded() const;
  /*!
   * \brief Sets if a reference contour is intruded.
   *
   * \param host_error_flag_reference_contour_intruded If a reference contour is intruded.
   */
  void setHostErrorFlagReferenceContourIntruded(bool host_error_flag_reference_contour_intruded);

  /*!
   * \brief Gets if a critical error is present.
   *
   * \returns If a critical error is present.
   */
  bool getHostErrorFlagCriticalError() const;
  /*!
   * \brief Sets if a critical error is present.
   *
   * \param host_error_flag_critical_error If a critical error is present.
   */
  void setHostErrorFlagCriticalError(bool host_error_flag_critical_error);

  /*!
   * \brief Gets the first linear velocity output.
   *
   * \returns The first linear velocity output.
   */
  int16_t getVelocity0() const;
  /*!
   * \brief Sets the first linear velocity output.
   *
   * \param velocity_0 The first linear velocity output.
   */
  void setVelocity0(const int16_t& velocity_0);

  /*!
   * \brief Gets the second linear velocity output.
   *
   * \returns The second linear velocity output.
   */
  int16_t getVelocity1() const;
  /*!
   * \brief Sets the second linear velocity output.
   *
   * \param velocity_1 The second linear velocity output.
   */
  void setVelocity1(const int16_t& velocity_1);

  /*!
   * \brief Gets if the first linear velocity output is valid.
   *
   * \returns If the first linear velocity output is valid.
   */
  bool getVelocity0Valid() const;
  /*!
   * \brief Sets if the first linear velocity output is valid.
   *
   * \param velocity_0_valid If the first linear velocity output is valid.
   */
  void setVelocity0Valid(bool velocity_0_valid);

  /*!
   * \brief Gets if the second linear velocity output is valid.
   *
   * \returns If the second linear velocity output is valid.
   */
  bool getVelocity1Valid() const;
  /*!
   * \brief Sets if the second linear velocity output is valid.
   *
   * \param velocity_1_valid If the second linear velocity output is valid.
   */
  void setVelocity1Valid(bool velocity_1_valid);

  /*!
   * \brief Gets if the first linear velocity output is transmitted safely.
   *
   * \returns If the first linear velocity output is transmitted safely.
   */
  bool getVelocity0TransmittedSafely() const;
  /*!
   * \brief Sets if the first linear velocity output is transmitted safely.
   *
   * \param velocity_0_transmitted_safely If the first linear velocity output is transmitted safely.
   */
  void setVelocity0TransmittedSafely(bool velocity_0_transmitted_safely);

  /*!
   * \brief Gets if the second linear velocity output is transmitted safely.
   *
   * \returns if the second linear velocity output is transmitted safely.
   */
  bool getVelocity1TransmittedSafely() const;
  /*!
   * \brief Sets if the second linear velocity output is transmitted safely.
   *
   * \param velocity_1_transmitted_safely if the second linear velocity output is transmitted
   * safely.
   */
  void setVelocity1TransmittedSafely(bool velocity_1_transmitted_safely);

  /*!
   * \brief Gets the resulting velocity for each monitoring case table.
   *
   * \returns  The resulting velocity for each monitoring case table.
   */
  std::vector<int16_t> getResultingVelocityVector() const;
  /*!
   * \brief Sets the resulting velocity for each monitoring case table.
   *
   * \param resulting_velocity_vector The resulting velocity for each monitoring case table.
   */
  void setResultingVelocityVector(const std::vector<int16_t>& resulting_velocity_vector);

  /*!
   * \brief Gets if the resulting velocities are valid.
   *
   * \returns  If the resulting velocities are valid.
   */
  std::vector<bool> getResultingVelocityIsValidVector() const;
  void
    /*!
     * \brief Sets if the resulting velocities are valid.
     *
     * \param resulting_velocity_is_valid_vector If the resulting velocities are valid.
     */
  setResultingVelocityIsValidVector(const std::vector<bool>& resulting_velocity_is_valid_vector);

  /*!
   * \brief Gets if the sleep mode is valid.
   *
   * \returns If the sleep mode is valid.
   */
  bool getFlagsSleepModeOutputIsValid() const;
  /*!
   * \brief Sets if the sleep mode is valid.
   *
   * \param flags_sleep_mode_output_is_valid If the sleep mode is valid.
   */
  void setFlagsSleepModeOutputIsValid(bool flags_sleep_mode_output_is_valid);

  /*!
   * \brief Gets if the error flags are valid.
   *
   * \returns If the error flags are valid.
   */
  bool getFlagsHostErrorFlagsAreValid() const;
  /*!
   * \brief Sets if the error flags are valid.
   *
   * \param flags_host_error_flags_are_valid If the error flags are valid.
   */
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


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONOUTPUTS_H
