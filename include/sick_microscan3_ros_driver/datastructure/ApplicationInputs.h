#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class ApplicationInputs
{
public:
  ApplicationInputs();

  std::vector<bool> getUnsafeInputsInputSources() const;
  void setUnsafeInputsInputSources(const std::vector<bool> &unsafe_inputs_input_sources);

  std::vector<bool> getUnsafeInputsFlags() const;
  void setUnsafeInputsFlags(const std::vector<bool> &unsafe_inputs_flags);

  std::vector<UINT16> getMonitoringCase() const;
  void setMonitoringCase(const std::vector<UINT16> &monitoring_case);

  std::vector<bool> getMonitoringCaseFlags() const;
  void setMonitoringCaseFlags(const std::vector<bool> &monitoring_case_flags);

  INT16 getVelocity0() const;
  void setVelocity0(const INT16 &velocity_0);

  INT16 getVelocity1() const;
  void setVelocity1(const INT16 &velocity_1);

  bool getVelocity0Valid() const;
  void setVelocity0Valid(bool velocity_0_valid);

  bool getVelocity1Valid() const;
  void setVelocity1Valid(bool velocity_1_valid);

  bool getVelocity0TransmittedSafely() const;
  void setVelocity0TransmittedSafely(bool velocity_0_transmitted_safely);

  bool getVelocity1TransmittedSafely() const;
  void setVelocity1TransmittedSafely(bool velocity_1_transmitted_safely);

  INT8 getSleepModeInput() const;
  void setSleepModeInput(const INT8 &sleep_mode_input);

private:
  std::vector<bool> m_unsafe_inputs_input_sources; //TODO in struct??? with next
  std::vector<bool> m_unsafe_inputs_flags;

  std::vector<UINT16> m_monitoring_case; //TODO 20 cases why 32 bit flags?
  std::vector<bool> m_monitoring_case_flags;

  INT16 m_velocity_0;
  INT16 m_velocity_1;

  bool m_velocity_0_valid;
  bool m_velocity_1_valid;
  bool m_velocity_0_transmitted_safely;
  bool m_velocity_1_transmitted_safely;

  INT8 m_sleep_mode_input; //TODO enum8
};

}
}
