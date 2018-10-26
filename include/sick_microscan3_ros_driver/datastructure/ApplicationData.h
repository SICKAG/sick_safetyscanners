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

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_APPLICATIONDATA_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_APPLICATIONDATA_H

#include <sick_microscan3_ros_driver/datastructure/ApplicationInputs.h>
#include <sick_microscan3_ros_driver/datastructure/ApplicationOutputs.h>

namespace sick {
namespace datastructure {

class ApplicationData
{
public:
  ApplicationData();

  ApplicationInputs getInputs() const;
  void setInputs(const ApplicationInputs& inputs);

  ApplicationOutputs getOutputs() const;
  void setOutputs(const ApplicationOutputs& outputs);

  bool isEmpty() const;
  void setIsEmpty(bool is_empty);

private:
  bool m_is_empty;

  ApplicationInputs m_inputs;
  ApplicationOutputs m_outputs;
};


}  // namespace datastructure
}  // namespace sick

#endif
