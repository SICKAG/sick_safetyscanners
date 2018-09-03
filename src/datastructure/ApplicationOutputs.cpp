#include <sick_microscan3_ros_driver/datastructure/ApplicationOutputs.h>

namespace sick {
namespace datastructure {

ApplicationOutputs::ApplicationOutputs()
{

}

std::vector<bool> ApplicationOutputs::getEvalOut() const
{
  return m_eval_out;
}

void ApplicationOutputs::setEvalOut(const std::vector<bool> &eval_out)
{
  m_eval_out = eval_out;
}

std::vector<bool> ApplicationOutputs::getEvalOutIsSafe() const
{
  return m_eval_out_is_safe;
}

void ApplicationOutputs::setEvalOutIsSafe(const std::vector<bool> &eval_out_is_safe)
{
  m_eval_out_is_safe = eval_out_is_safe;
}

std::vector<bool> ApplicationOutputs::getEvalOutIsValid() const
{
  return m_eval_out_is_valid;
}

void ApplicationOutputs::setEvalOutIsValid(const std::vector<bool> &eval_out_is_valid)
{
  m_eval_out_is_valid = eval_out_is_valid;
}

std::vector<UINT16> ApplicationOutputs::getMonitoringCase() const
{
  return m_monitoring_case;
}

void ApplicationOutputs::setMonitoringCase(const std::vector<UINT16> &monitoring_case)
{
  m_monitoring_case = monitoring_case;
}

std::vector<bool> ApplicationOutputs::getMonitoringCaseFlags() const
{
  return m_monitoring_case_flags;
}

void ApplicationOutputs::setMonitoringCaseFlags(const std::vector<bool> &monitoring_case_flags)
{
  m_monitoring_case_flags = monitoring_case_flags;
}

INT8 ApplicationOutputs::getSleepModeOutput() const
{
  return m_sleep_mode_output;
}

void ApplicationOutputs::setSleepModeOutput(const INT8 &sleep_mode_output)
{
  m_sleep_mode_output = sleep_mode_output;
}

bool ApplicationOutputs::getHostErrorFlagContaminationWarning() const
{
  return m_host_error_flag_contamination_warning;
}

void ApplicationOutputs::setHostErrorFlagContaminationWarning(bool host_error_flag_contamination_warning)
{
  m_host_error_flag_contamination_warning = host_error_flag_contamination_warning;
}

bool ApplicationOutputs::getHostErrorFlagContaminationError() const
{
  return m_host_error_flag_contamination_error;
}

void ApplicationOutputs::setHostErrorFlagContaminationError(bool host_error_flag_contamination_error)
{
  m_host_error_flag_contamination_error = host_error_flag_contamination_error;
}

bool ApplicationOutputs::getHostErrorFlagManipulationError() const
{
  return m_host_error_flag_manipulation_error;
}

void ApplicationOutputs::setHostErrorFlagManipulationError(bool host_error_flag_manipulation_error)
{
  m_host_error_flag_manipulation_error = host_error_flag_manipulation_error;
}

bool ApplicationOutputs::getHostErrorFlagGlare() const
{
  return m_host_error_flag_glare;
}

void ApplicationOutputs::setHostErrorFlagGlare(bool host_error_flag_glare)
{
  m_host_error_flag_glare = host_error_flag_glare;
}

bool ApplicationOutputs::getHostErrorFlagReferenceContourIntruded() const
{
  return m_host_error_flag_reference_contour_intruded;
}

void ApplicationOutputs::setHostErrorFlagReferenceContourIntruded(bool host_error_flag_reference_contour_intruded)
{
  m_host_error_flag_reference_contour_intruded = host_error_flag_reference_contour_intruded;
}

bool ApplicationOutputs::getHostErrorFlagCriticalError() const
{
  return m_host_error_flag_critical_error;
}

void ApplicationOutputs::setHostErrorFlagCriticalError(bool host_error_flag_critical_error)
{
  m_host_error_flag_critical_error = host_error_flag_critical_error;
}

INT16 ApplicationOutputs::getVelocity0() const
{
  return m_velocity_0;
}

void ApplicationOutputs::setVelocity0(const INT16 &velocity_0)
{
  m_velocity_0 = velocity_0;
}

INT16 ApplicationOutputs::getVelocity1() const
{
  return m_velocity_1;
}

void ApplicationOutputs::setVelocity1(const INT16 &velocity_1)
{
  m_velocity_1 = velocity_1;
}

bool ApplicationOutputs::getVelocity0Valid() const
{
  return m_velocity_0_valid;
}

void ApplicationOutputs::setVelocity0Valid(bool velocity_0_valid)
{
  m_velocity_0_valid = velocity_0_valid;
}

bool ApplicationOutputs::getVelocity1Valid() const
{
  return m_velocity_1_valid;
}

void ApplicationOutputs::setVelocity1Valid(bool velocity_1_valid)
{
  m_velocity_1_valid = velocity_1_valid;
}

bool ApplicationOutputs::getVelocity0TransmittedSafely() const
{
  return m_velocity_0_transmitted_safely;
}

void ApplicationOutputs::setVelocity0TransmittedSafely(bool velocity_0_transmitted_safely)
{
  m_velocity_0_transmitted_safely = velocity_0_transmitted_safely;
}

bool ApplicationOutputs::getVelocity1TransmittedSafely() const
{
  return m_velocity_1_transmitted_safely;
}

void ApplicationOutputs::setVelocity1TransmittedSafely(bool velocity_1_transmitted_safely)
{
  m_velocity_1_transmitted_safely = velocity_1_transmitted_safely;
}

std::vector<INT16> ApplicationOutputs::getResultingVelocity() const
{
  return m_resulting_velocity;
}

void ApplicationOutputs::setResultingVelocity(const std::vector<INT16> &resulting_velocity)
{
  m_resulting_velocity = resulting_velocity;
}

std::vector<bool> ApplicationOutputs::getResultingVelocityIsValid() const
{
  return m_resulting_velocity_is_valid;
}

void ApplicationOutputs::setResultingVelocityIsValid(const std::vector<bool> &resulting_velocity_is_valid)
{
  m_resulting_velocity_is_valid = resulting_velocity_is_valid;
}

bool ApplicationOutputs::getFlagsSleepModeOutputIsValid() const
{
  return m_flags_sleep_mode_output_is_valid;
}

void ApplicationOutputs::setFlagsSleepModeOutputIsValid(bool flags_sleep_mode_output_is_valid)
{
  m_flags_sleep_mode_output_is_valid = flags_sleep_mode_output_is_valid;
}

bool ApplicationOutputs::getFlagsHostErrorFlagsAreValid() const
{
  return m_flags_host_error_flags_are_valid;
}

void ApplicationOutputs::setFlagsHostErrorFlagsAreValid(bool flags_host_error_flags_are_valid)
{
  m_flags_host_error_flags_are_valid = flags_host_error_flags_are_valid;
}

}
}

