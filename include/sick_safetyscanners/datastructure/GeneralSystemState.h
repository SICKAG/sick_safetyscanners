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
 * \file GeneralSystemState.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_GENERALSYSTEMSTATE_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_GENERALSYSTEMSTATE_H

#include <stdint.h>
#include <vector>

namespace sick {
namespace datastructure {

/*!
 * \brief The GeneralSystemState class. It includes a summary of the current system state,
 *  the state of the safe and the non-safe cut-off path and bits indicating if a restart
 * interlock is ready to be reset.
 */
class GeneralSystemState
{
public:
  /*!
   * \brief Constructor of GeneralSystemState, creates empty instance.
   */
  GeneralSystemState();

  /*!
   * \brief Returns if run mode is active.
   * \return If run mode is active.
   */
  bool getRunModeActive() const;
  /*!
   * \brief Setter for run mode.
   * \param run_mode_active If run mode is active or inactive.
   */
  void setRunModeActive(bool run_mode_active);

  /*!
   * \brief Returns if the standby mode is active.
   * \return If the standby mode is active.
   */
  bool getStandbyModeActive() const;

  /*!
   * \brief Setter for standby mode.
   * \param standby_mode_active If standby mode is active or inactive.
   */
  void setStandbyModeActive(bool standby_mode_active);

  /*!
   * \brief Returns if a contamination warning is exists.
   * \return If a contamination warning exists.
   */
  bool getContaminationWarning() const;

  /*!
   * \brief Set if a contamination warning exists.
   * \param contamination_warning If a contamination warning exists.
   */
  void setContaminationWarning(bool contamination_warning);

  /*!
   * \brief Returns if a contamination error exists.
   * \return If a contamination error exists.
   */
  bool getContaminationError() const;

  /*!
   * \brief Set if a contamination error exists.
   * \param contamination_error If a contamination error exists.
   */
  void setContaminationError(bool contamination_error);

  /*!
   * \brief Returns if the reference contour status is true.
   * \return If the reference contour status ist true.
   */
  bool getReferenceContourStatus() const;

  /*!
   * \brief Set the reference contour status.
   * \param reference_contour_status The new reference contour status.
   */
  void setReferenceContourStatus(bool reference_contour_status);

  /*!
   * \brief Returns if the manipulation status is set to true.
   * \return If the manipulation status is set.
   */
  bool getManipulationStatus() const;

  /*!
   * \brief Set the manipulation status.
   * \param manipulation_status The new manipulation status.
   */
  void setManipulationStatus(bool manipulation_status);

  /*!
   * \brief Returns the state for all safe cut off paths.
   * \return Vector containing the state of all safe cut off paths.
   */
  std::vector<bool> getSafeCutOffPathVector() const;

  /*!
   * \brief Sets the state of all safe cut-off paths.
   * \param safe_cut_off_path_vector Vector for the state of all safe cut-off paths.
   */
  void setSafeCutOffPathvector(const std::vector<bool>& safe_cut_off_path_vector);

  /*!
   * \brief Returns the state of all non-safe cut-off paths.
   * \return  Vector containing the state of all non-safe cut-off paths
   */
  std::vector<bool> getNonSafeCutOffPathVector() const;

  /*!
   * \brief Sets the state of all non-safe cut-off paths.
   * \param non_safe_cut_off_path_vector Vector for the state of all non-safe cut-off paths
   */
  void setNonSafeCutOffPathVector(const std::vector<bool>& non_safe_cut_off_path_vector);

  /*!
   * \brief Returns if a cut-off path has to be reset.
   * \return Vector if a cut-off path has to be reset.
   */
  std::vector<bool> getResetRequiredCutOffPathVector() const;

  /*!
   * \brief Sets the reset state for all cut-off paths
   * \param reset_required_cut_off_path_vector Vector for the reset state of all cut-off paths
   */
  void
  setResetRequiredCutOffPathVector(const std::vector<bool>& reset_required_cut_off_path_vector);

  /*!
   * \brief Returns the current monitor case table 1.
   * \return The current monitoring case table 1.
   */
  uint8_t getCurrentMonitoringCaseNoTable1() const;

  /*!
   * \brief Sets the current monitoring case table 1.
   * \param current_monitoring_case_no_table_1 The current monitoring case table 1.
   */
  void setCurrentMonitoringCaseNoTable1(const uint8_t& current_monitoring_case_no_table_1);

  /*!
   * \brief Returns the current monitor case table 2.
   * \return The current monitoring case table 2.
   */
  uint8_t getCurrentMonitoringCaseNoTable2() const;

  /*!
   * \brief Sets the current monitoring case table 2.
   * \param current_monitoring_case_no_table_2 The current monitoring case table 2.
   */
  void setCurrentMonitoringCaseNoTable2(const uint8_t& current_monitoring_case_no_table_2);

  /*!
   * \brief Returns the current monitor case table 3.
   * \return The current monitoring case table 3.
   */
  uint8_t getCurrentMonitoringCaseNoTable3() const;

  /*!
   * \brief Sets the current monitoring case table 3.
   * \param current_monitoring_case_no_table_3 The current monitoring case table 3.
   */
  void setCurrentMonitoringCaseNoTable3(const uint8_t& current_monitoring_case_no_table_3);

  /*!
   * \brief Returns the current monitor case table 4.
   * \return The current monitoring case table 4.
   */
  uint8_t getCurrentMonitoringCaseNoTable4() const;

  /*!
   * \brief Sets the current monitoring case table 4.
   * \param current_monitoring_case_no_table_4 The current monitoring case table 4 .
   */
  void setCurrentMonitoringCaseNoTable4(const uint8_t& current_monitoring_case_no_table_4);

  /*!
   * \brief Return if an application error exists.
   * \return If an application error exists.
   */
  bool getApplicationError() const;

  /*!
   * \brief Set if an application error exists.
   * \param application_error If an application error exists.
   */
  void setApplicationError(bool application_error);

  /*!
   * \brief Return if a device error exists.
   * \return If a device error exists.
   */
  bool getDeviceError() const;

  /*!
   * \brief Set if a device error exists.
   * \param device_error If a device error exists
   */
  void setDeviceError(bool device_error);


  /*!
   * \brief Return if general system state has been enabled.
   * \return If general system state has been enabled.
   */
  bool isEmpty() const;

  /*!
   * \brief Set if general system state has been enabled.
   * \param is_empty If general system state has been enabled.
   */
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;
  bool m_run_mode_active;
  bool m_standby_mode_active;
  bool m_contamination_warning;
  bool m_contamination_error;
  bool m_reference_contour_status;
  bool m_manipulation_status;

  std::vector<bool> m_safe_cut_off_path_vector;
  std::vector<bool> m_non_safe_cut_off_path_vector;
  std::vector<bool> m_reset_required_cut_off_path_vector;

  uint8_t m_current_monitoring_case_no_table_1;
  uint8_t m_current_monitoring_case_no_table_2;
  uint8_t m_current_monitoring_case_no_table_3;
  uint8_t m_current_monitoring_case_no_table_4;

  bool m_application_error;
  bool m_device_error;
};

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_GENERALSYSTEMSTATE_H
