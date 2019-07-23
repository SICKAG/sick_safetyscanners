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
 * \file GeneralSystemState.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/GeneralSystemState.h>

namespace sick {
namespace datastructure {

GeneralSystemState::GeneralSystemState()
  : m_is_empty(false)
{
}

bool GeneralSystemState::getRunModeActive() const
{
  return m_run_mode_active;
}

void GeneralSystemState::setRunModeActive(bool run_mode_active)
{
  m_run_mode_active = run_mode_active;
}

bool GeneralSystemState::getStandbyModeActive() const
{
  return m_standby_mode_active;
}

void GeneralSystemState::setStandbyModeActive(bool standby_mode_active)
{
  m_standby_mode_active = standby_mode_active;
}

bool GeneralSystemState::getContaminationWarning() const
{
  return m_contamination_warning;
}

void GeneralSystemState::setContaminationWarning(bool contamination_warning)
{
  m_contamination_warning = contamination_warning;
}

bool GeneralSystemState::getContaminationError() const
{
  return m_contamination_error;
}

void GeneralSystemState::setContaminationError(bool contamination_error)
{
  m_contamination_error = contamination_error;
}

bool GeneralSystemState::getReferenceContourStatus() const
{
  return m_reference_contour_status;
}

void GeneralSystemState::setReferenceContourStatus(bool reference_contour_status)
{
  m_reference_contour_status = reference_contour_status;
}

bool GeneralSystemState::getManipulationStatus() const
{
  return m_manipulation_status;
}

void GeneralSystemState::setManipulationStatus(bool manipulation_status)
{
  m_manipulation_status = manipulation_status;
}

std::vector<bool> GeneralSystemState::getSafeCutOffPathVector() const
{
  return m_safe_cut_off_path_vector;
}

void GeneralSystemState::setSafeCutOffPathvector(const std::vector<bool>& safe_cut_off_path_vector)
{
  m_safe_cut_off_path_vector = safe_cut_off_path_vector;
}

std::vector<bool> GeneralSystemState::getNonSafeCutOffPathVector() const
{
  return m_non_safe_cut_off_path_vector;
}

void GeneralSystemState::setNonSafeCutOffPathVector(
  const std::vector<bool>& non_safe_cut_off_path_vector)
{
  m_non_safe_cut_off_path_vector = non_safe_cut_off_path_vector;
}

std::vector<bool> GeneralSystemState::getResetRequiredCutOffPathVector() const
{
  return m_reset_required_cut_off_path_vector;
}

void GeneralSystemState::setResetRequiredCutOffPathVector(
  const std::vector<bool>& reset_required_cut_off_path_vector)
{
  m_reset_required_cut_off_path_vector = reset_required_cut_off_path_vector;
}

uint8_t GeneralSystemState::getCurrentMonitoringCaseNoTable1() const
{
  return m_current_monitoring_case_no_table_1;
}

void GeneralSystemState::setCurrentMonitoringCaseNoTable1(
  const uint8_t& current_monitoring_case_no_table_1)
{
  m_current_monitoring_case_no_table_1 = current_monitoring_case_no_table_1;
}

uint8_t GeneralSystemState::getCurrentMonitoringCaseNoTable2() const
{
  return m_current_monitoring_case_no_table_2;
}

void GeneralSystemState::setCurrentMonitoringCaseNoTable2(
  const uint8_t& current_monitoring_case_no_table_2)
{
  m_current_monitoring_case_no_table_2 = current_monitoring_case_no_table_2;
}

uint8_t GeneralSystemState::getCurrentMonitoringCaseNoTable3() const
{
  return m_current_monitoring_case_no_table_3;
}

void GeneralSystemState::setCurrentMonitoringCaseNoTable3(
  const uint8_t& current_monitoring_case_no_table_3)
{
  m_current_monitoring_case_no_table_3 = current_monitoring_case_no_table_3;
}

uint8_t GeneralSystemState::getCurrentMonitoringCaseNoTable4() const
{
  return m_current_monitoring_case_no_table_4;
}

void GeneralSystemState::setCurrentMonitoringCaseNoTable4(
  const uint8_t& current_monitoring_case_no_table_4)
{
  m_current_monitoring_case_no_table_4 = current_monitoring_case_no_table_4;
}

bool GeneralSystemState::getApplicationError() const
{
  return m_application_error;
}

void GeneralSystemState::setApplicationError(bool application_error)
{
  m_application_error = application_error;
}

bool GeneralSystemState::getDeviceError() const
{
  return m_device_error;
}

void GeneralSystemState::setDeviceError(bool device_error)
{
  m_device_error = device_error;
}

bool GeneralSystemState::isEmpty() const
{
  return m_is_empty;
}

void GeneralSystemState::setIsEmpty(bool is_empty)
{
  m_is_empty = is_empty;
}


} // namespace datastructure
} // namespace sick
