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

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONINPUTS_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONINPUTS_H

#include <stdint.h>
#include <vector>

namespace sick {
namespace datastructure {

/*!
 * \brief The applications inputs from a udp data packet.
 */
class ApplicationInputs
{
public:
  /*!
   * \brief Constructor of application inputs.
   */
  ApplicationInputs();

  /*!
   * \brief Gets the unsafe input sources.
   *
   * The individual bits represent the current state of the static input sources used for
   * monitoring case switching
   *
   * \returns The unsafe input sources.
   */
  std::vector<bool> getUnsafeInputsInputSourcesVector() const;
  void
    /*!
     * \brief Sets the unsafe input sources.
     *
     * \param unsafe_inputs_input_sources_vector The unsafe input sources.
     */
  setUnsafeInputsInputSourcesVector(const std::vector<bool>& unsafe_inputs_input_sources_vector);

  /*!
   * \brief Gets the flags for the unsafe input sources.
   *
   * There is one flag (bit) for each static input
   * source. If the flag is HIGH, the corresponding input
   * source is available to be used by the application.
   *
   * \returns The unsafe input sources flags.
   */
  std::vector<bool> getUnsafeInputsFlagsVector() const;
  /*!
   * \brief Sets the unsafe input sources flags.
   *
   * \param unsafe_inputs_flags_vector The unsafe input sources flags.
   */
  void setUnsafeInputsFlagsVector(const std::vector<bool>& unsafe_inputs_flags_vector);

  /*!
   * \brief Gets the monitoring case numbers.
   *
   * \returns The monitoring case vector.
   */
  std::vector<uint16_t> getMonitoringCasevector() const;
  /*!
   * \brief Sets the monitoring case vector.
   *
   * \param monitoring_case_vector The monitoring case vector.
   */
  void setMonitoringCaseVector(const std::vector<uint16_t>& monitoring_case_vector);

  /*!
   * \brief Gets the monitoring case flags.
   *
   * \returns The monitoring case flags.
   */
  std::vector<bool> getMonitoringCaseFlagsVector() const;
  /*!
   * \brief Sets the monitoring case flags.
   *
   * \param monitoring_case_flags_vector The monitoring case flags.
   */
  void setMonitoringCaseFlagsVector(const std::vector<bool>& monitoring_case_flags_vector);

  /*!
   * \brief Gets the first linear velocity input.
   *
   * \returns The first linear velocity input.
   */
  int16_t getVelocity0() const;
  /*!
   * \brief Sets the first linear velocity input.
   *
   * \param velocity_0 The first linear velocity input.
   */
  void setVelocity0(const int16_t& velocity_0);

  /*!
   * \brief Gets the second linear velocity input.
   *
   * \returns The second linear velocity input
   */
  int16_t getVelocity1() const;
  /*!
   * \brief Sets the second linear velocity input.
   *
   * \param velocity_1 The second linear velocity input.
   */
  void setVelocity1(const int16_t& velocity_1);

  /*!
   * \brief Gets if first linear velocity input is valid.
   *
   * \returns  If first linear velocity input is valid.
   */
  bool getVelocity0Valid() const;
  /*!
   * \brief Sets if first linear velocity input is valid.
   *
   * \param velocity_0_valid If first linear velocity input is valid.
   */
  void setVelocity0Valid(bool velocity_0_valid);

  /*!
   * \brief Gets if second linear velocity input is valid.
   *
   * \returns If second linear velocity input is valid.
   */
  bool getVelocity1Valid() const;
  /*!
   * \brief If second linear velocity input is valid.
   *
   * \param velocity_1_valid If second linear velocity input is valid.
   */
  void setVelocity1Valid(bool velocity_1_valid);

  /*!
   * \brief Gets if first linear velocity input is transmitted safely.
   *
   * \returns If first linear velocity input is transmitted safely.
   */
  bool getVelocity0TransmittedSafely() const;
  /*!
   * \brief Sets if first linear velocity input is transmitted safely.
   *
   * \param velocity_0_transmitted_safely
   */
  void setVelocity0TransmittedSafely(bool velocity_0_transmitted_safely);

  /*!
   * \brief Gets if second linear velocity input is transmitted safely.
   *
   * \returns If second linear velocity input is transmitted safely.
   */
  bool getVelocity1TransmittedSafely() const;
  /*!
   * \brief Sets if second linear velocity input is transmitted safely.
   *
   * \param velocity_1_transmitted_safely If second linear velocity input is transmitted safely.
   */
  void setVelocity1TransmittedSafely(bool velocity_1_transmitted_safely);

  /*!
   * \brief Gets the state of the sleep mode.
   *
   * \returns The state of the sleep mode.
   */
  int8_t getSleepModeInput() const;
  /*!
   * \brief Sets the state of the sleep mode.
   *
   * \param sleep_mode_input The state of the sleep mode.
   */
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

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONINPUTS_H
