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
  void setSafeCutOffPathvector(const std::vector<bool> &safe_cut_off_path_vector);

  std::vector<bool> getNonSafeCutOffPathvector() const;
  void setNonSafeCutOffPathVector(const std::vector<bool> &non_safe_cut_off_path_vector);

  std::vector<bool> getResetRequiredCutOffPathVector() const;
  void setResetRequiredCutOffPathVector(const std::vector<bool> &reset_required_cut_off_path_vector);

  BYTE getCurrentMonitoringCaseNoTable_1() const;
  void setCurrentMonitoringCaseNoTable_1(const BYTE &current_monitoring_case_no_table_1);

  BYTE getCurrentMonitoringCaseNoTable_2() const;
  void setCurrentMonitoringCaseNoTable_2(const BYTE &current_monitoring_case_no_table_2);

  BYTE getCurrentMonitoringCaseNoTable_3() const;
  void setCurrentMonitoringCaseNoTable_3(const BYTE &current_monitoring_case_no_table_3);

  BYTE getCurrentMonitoringCaseNoTable_4() const;
  void setCurrentMonitoringCaseNoTable_4(const BYTE &current_monitoring_case_no_table_4);

  bool getApplicationError() const;
  void setApplicationError(bool application_error);

  bool getDeviceError() const;
  void setDeviceError(bool device_error);



private:
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

}
}
