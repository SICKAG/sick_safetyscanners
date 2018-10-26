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
 * \file ParseApplicationData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef PARSEAPPLICATIONDATA_H
#define PARSEAPPLICATIONDATA_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>


namespace sick {
namespace data_processing {

class ParseApplicationData
{
public:
  ParseApplicationData();
  datastructure::ApplicationData parseUDPSequence(const sick::datastructure::PacketBuffer& buffer,
                                                  datastructure::Data& data) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  void setDataInApplicationData(const uint8_t*& data_ptr,
                                datastructure::ApplicationData& application_data) const;
  void
  setApplicationInputsInApplicationData(const uint8_t*& data_ptr,
                                        datastructure::ApplicationData& application_data) const;
  void
  setApplicationOutputsInApplicationData(const uint8_t*& data_ptr,
                                         datastructure::ApplicationData& application_data) const;
  void setDataInApplicationInputs(const uint8_t*& data_ptr,
                                  datastructure::ApplicationInputs& inputs) const;
  void setDataInApplicationOutputs(const uint8_t*& data_ptr,
                                   datastructure::ApplicationOutputs& outputs) const;
  void setUnsafeInputsInApplicationInputs(const uint8_t*& data_ptr,
                                          datastructure::ApplicationInputs& inputs) const;
  void setUnsafeInputsSourcesInApplicationInputs(const uint8_t*& data_ptr,
                                                 datastructure::ApplicationInputs& inputs) const;
  void setUnsafeInputsFlagsInApplicationInputs(const uint8_t*& data_ptr,
                                               datastructure::ApplicationInputs& inputs) const;
  void setMonitoringCaseInputsInApplicationInputs(const uint8_t*& data_ptr,
                                                  datastructure::ApplicationInputs& inputs) const;
  void setMonitoringCaseNumbersInApplicationInputs(const uint8_t*& data_ptr,
                                                   datastructure::ApplicationInputs& inputs) const;
  void setMonitoringCaseFlagsInApplicationInputs(const uint8_t*& data_ptr,
                                                 datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocityInputsInApplicationInputs(const uint8_t*& data_ptr,
                                                  datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocity0InApplicationInputs(const uint8_t*& data_ptr,
                                             datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocity1InApplicationInputs(const uint8_t*& data_ptr,
                                             datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocityFlagsInApplicationInputs(const uint8_t*& data_ptr,
                                                 datastructure::ApplicationInputs& inputs) const;
  void setSleepModeInputInApplicationInputs(const uint8_t*& data_ptr,
                                            datastructure::ApplicationInputs& inputs) const;
  void
  setEvalutaionPathsOutputsInApplicationOutputs(const uint8_t*& data_ptr,
                                                datastructure::ApplicationOutputs& outputs) const;
  void setEvaluationPathsOutputsEvalOutInApplicationOutputs(
    const uint8_t*& data_ptr, datastructure::ApplicationOutputs& outputs) const;
  void setEvaluationPathsOutputsIsSafeInApplicationOutputs(
    const uint8_t*& data_ptr, datastructure::ApplicationOutputs& outputs) const;
  void setEvaluationPathsOutputsValidFlagsInApplicationOutputs(
    const uint8_t*& data_ptr, datastructure::ApplicationOutputs& outputs) const;
  void
  setMonitoringCaseOutputsInApplicationOutputs(const uint8_t*& data_ptr,
                                               datastructure::ApplicationOutputs& outputs) const;
  void
  setMonitoringCaseNumbersInApplicationOutputs(const uint8_t*& data_ptr,
                                               datastructure::ApplicationOutputs& outputs) const;
  void setMonitoringCaseFlagsInApplicationOutputs(const uint8_t*& data_ptr,
                                                  datastructure::ApplicationOutputs& outputs) const;
  void setSleepModeOutputInApplicationOutputs(const uint8_t*& data_ptr,
                                              datastructure::ApplicationOutputs& outputs) const;
  void setErrorFlagsInApplicationOutputs(const uint8_t*& data_ptr,
                                         datastructure::ApplicationOutputs& outputs) const;
  void
  setLinearVelocityOutoutsInApplicationOutputs(const uint8_t*& data_ptr,
                                               datastructure::ApplicationOutputs& outputs) const;
  void setLinearVelocity0InApplicationOutputs(const uint8_t*& data_ptr,
                                              datastructure::ApplicationOutputs& outputs) const;
  void setLinearVelocity1InApplicationOutputs(const uint8_t*& data_ptr,
                                              datastructure::ApplicationOutputs& outputs) const;
  void setLinearVelocityFlagsInApplicationOutputs(const uint8_t*& data_ptr,
                                                  datastructure::ApplicationOutputs& outputs) const;
  void
  setResultingVelocityOutputsInApplicationOutputs(const uint8_t*& data_ptr,
                                                  datastructure::ApplicationOutputs& outputs) const;
  void setResultingVelocityInApplicationOutputs(const uint8_t*& data_ptr,
                                                datastructure::ApplicationOutputs& outputs) const;
  void
  setResultingVelocityFlagsInApplicationOutputs(const uint8_t*& data_ptr,
                                                datastructure::ApplicationOutputs& outputs) const;
  void setOutputFlagsinApplicationOutput(const uint8_t*& data_ptr,
                                         datastructure::ApplicationOutputs& outputs) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfApplicationDataIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

} // namespace data_processing
} // namespace sick

#endif
