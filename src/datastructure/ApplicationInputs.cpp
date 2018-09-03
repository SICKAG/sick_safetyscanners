#include <sick_microscan3_ros_driver/datastructure/ApplicationInputs.h>

namespace sick {
namespace datastructure {

ApplicationInputs::ApplicationInputs()
{

}

std::vector<bool> ApplicationInputs::getUnsafeInputsInputSources() const
{
  return m_unsafe_inputs_input_sources;
}

void ApplicationInputs::setUnsafeInputsInputSources(const std::vector<bool> &unsafe_inputs_input_sources)
{
  m_unsafe_inputs_input_sources = unsafe_inputs_input_sources;
}

std::vector<bool> ApplicationInputs::getUnsafeInputsFlags() const
{
  return m_unsafe_inputs_flags;
}

void ApplicationInputs::setUnsafeInputsFlags(const std::vector<bool> &unsafe_inputs_flags)
{
  m_unsafe_inputs_flags = unsafe_inputs_flags;
}

std::vector<UINT16> ApplicationInputs::getMonitoringCase() const
{
  return m_monitoring_case;
}

void ApplicationInputs::setMonitoringCase(const std::vector<UINT16> &monitoring_case)
{
  m_monitoring_case = monitoring_case;
}

std::vector<bool> ApplicationInputs::getMonitoringCaseFlags() const
{
  return m_monitoring_case_flags;
}

void ApplicationInputs::setMonitoringCaseFlags(const std::vector<bool> &monitoring_case_flags)
{
  m_monitoring_case_flags = monitoring_case_flags;
}

INT16 ApplicationInputs::getVelocity0() const
{
  return m_velocity_0;
}

void ApplicationInputs::setVelocity0(const INT16 &velocity_0)
{
  m_velocity_0 = velocity_0;
}

INT16 ApplicationInputs::getVelocity1() const
{
  return m_velocity_1;
}

void ApplicationInputs::setVelocity1(const INT16 &velocity_1)
{
  m_velocity_1 = velocity_1;
}

bool ApplicationInputs::getVelocity0Valid() const
{
  return m_velocity_0_valid;
}

void ApplicationInputs::setVelocity0Valid(bool velocity_0_valid)
{
  m_velocity_0_valid = velocity_0_valid;
}

bool ApplicationInputs::getVelocity1Valid() const
{
  return m_velocity_1_valid;
}

void ApplicationInputs::setVelocity1Valid(bool velocity_1_valid)
{
  m_velocity_1_valid = velocity_1_valid;
}

bool ApplicationInputs::getVelocity0TransmittedSafely() const
{
  return m_velocity_0_transmitted_safely;
}

void ApplicationInputs::setVelocity0TransmittedSafely(bool velocity_0_transmitted_safely)
{
  m_velocity_0_transmitted_safely = velocity_0_transmitted_safely;
}

bool ApplicationInputs::getVelocity1TransmittedSafely() const
{
  return m_velocity_1_transmitted_safely;
}

void ApplicationInputs::setVelocity1TransmittedSafely(bool velocity_1_transmitted_safely)
{
  m_velocity_1_transmitted_safely = velocity_1_transmitted_safely;
}

INT8 ApplicationInputs::getSleepModeInput() const
{
  return m_sleep_mode_input;
}

void ApplicationInputs::setSleepModeInput(const INT8 &sleep_mode_input)
{
  m_sleep_mode_input = sleep_mode_input;
}

}
}

