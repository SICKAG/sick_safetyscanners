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
 * \file ApplicationData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONDATA_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONDATA_H

#include <sick_safetyscanners/datastructure/ApplicationInputs.h>
#include <sick_safetyscanners/datastructure/ApplicationOutputs.h>

namespace sick {
namespace datastructure {

/*!
 * \brief The application io class, bundles application input and output.
 */
class ApplicationData
{
public:
  /*!
   * \brief Constructor of the application data.
   */
  ApplicationData();

  /*!
   * \brief Gets the application input.
   *
   * \returns The application input.
   */
  ApplicationInputs getInputs() const;
  /*!
   * \brief Sets the application input.
   *
   * \param inputs The new application input.
   */
  void setInputs(const ApplicationInputs& inputs);

  /*!
   * \brief Gets the application output.
   *
   * \returns The application output.
   */
  ApplicationOutputs getOutputs() const;
  /*!
   * \brief Sets the application output.
   *
   * \param outputs The application output.
   */
  void setOutputs(const ApplicationOutputs& outputs);

  /*!
   * \brief Check if application data is empty.
   *
   * \returns If application data is empty.
   */
  bool isEmpty() const;
  /*!
   * \brief Sets if application data is empty.
   *
   * \param is_empty If application data is empty.
   */
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;

  ApplicationInputs m_inputs;
  ApplicationOutputs m_outputs;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_APPLICATIONDATA_H
