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

#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class GeneralSystemState
{
public:
  GeneralSystemState();

  bool getRunModeActive() const;
  void setRunModeActive(bool run_mode_active);

  bool getStandbyModeActive() const;
  void setStandbyModeActive(bool standby_mode_active);

  bool getContaminationWarning() const;
  void setContaminationWarning(bool contamination_warning);

  bool getContaminationError() const;
  void setContaminationError(bool contamination_error);

  bool getReferenceContourStatus() const;
  void setReferenceContourStatus(bool reference_contour_status);

  bool getManipulationStatus() const;
  void setManipulationStatus(bool manipulation_status);

  std::vector<bool> getSafeCutOffPathVector() const;
  void setSafeCutOffPathvector(const std::vector<bool>& safe_cut_off_path_vector);

  std::vector<bool> getNonSafeCutOffPathVector() const;
  void setNonSafeCutOffPathVector(const std::vector<bool>& non_safe_cut_off_path_vector);

  std::vector<bool> getResetRequiredCutOffPathVector() const;
  void
  setResetRequiredCutOffPathVector(const std::vector<bool>& reset_required_cut_off_path_vector);

  BYTE getCurrentMonitoringCaseNoTable_1() const;
  void setCurrentMonitoringCaseNoTable_1(const BYTE& current_monitoring_case_no_table_1);

  BYTE getCurrentMonitoringCaseNoTable_2() const;
  void setCurrentMonitoringCaseNoTable_2(const BYTE& current_monitoring_case_no_table_2);

  BYTE getCurrentMonitoringCaseNoTable_3() const;
  void setCurrentMonitoringCaseNoTable_3(const BYTE& current_monitoring_case_no_table_3);

  BYTE getCurrentMonitoringCaseNoTable_4() const;
  void setCurrentMonitoringCaseNoTable_4(const BYTE& current_monitoring_case_no_table_4);

  bool getApplicationError() const;
  void setApplicationError(bool application_error);

  bool getDeviceError() const;
  void setDeviceError(bool device_error);


  bool isEmpty() const;
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

  BYTE m_current_monitoring_case_no_table_1;
  BYTE m_current_monitoring_case_no_table_2;
  BYTE m_current_monitoring_case_no_table_3;
  BYTE m_current_monitoring_case_no_table_4;

  bool m_application_error;
  bool m_device_error;
};

} // namespace datastructure
} // namespace sick
