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

#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseApplicationData : public AbstractParseUDPSequence
{
public:
  ParseApplicationData();
  datastructure::ApplicationData parseUDPSequence(sick::datastructure::PacketBuffer buffer,
                                                  datastructure::Data& data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  void setDataInApplicationData(const BYTE* data_ptr,
                                datastructure::ApplicationData& application_data);
  void setApplicationInputsInApplicationData(const BYTE* data_ptr,
                                             datastructure::ApplicationData& application_data);
  void setApplicationOutputsInApplicationData(const BYTE* data_ptr,
                                              datastructure::ApplicationData& application_data);
  void setDataInApplicationInputs(const BYTE* data_ptr, datastructure::ApplicationInputs& inputs);
  void setDataInApplicationOutputs(const BYTE* data_ptr,
                                   datastructure::ApplicationOutputs& outputs);
  void setUnsafeInputsInApplicationInputs(const BYTE* data_ptr,
                                          datastructure::ApplicationInputs& inputs);
  void setUnsafeInputsSourcesInApplicationInputs(const BYTE* data_ptr,
                                                 datastructure::ApplicationInputs& inputs);
  void setUnsafeInputsFlagsInApplicationInputs(const BYTE* data_ptr,
                                               datastructure::ApplicationInputs& inputs);
  void setMonitoringCaseInputsInApplicationInputs(const BYTE* data_ptr,
                                                  datastructure::ApplicationInputs& inputs);
  void setMonitoringCaseNumbersInApplicationInputs(const BYTE* data_ptr,
                                                   datastructure::ApplicationInputs& inputs);
  void setMonitoringCaseFlagsInApplicationInputs(const BYTE* data_ptr,
                                                 datastructure::ApplicationInputs& inputs);
  void setLinearVelocityInputsInApplicationInputs(const BYTE* data_ptr,
                                                  datastructure::ApplicationInputs& inputs);
  void setLinearVelocity0InApplicationInputs(const BYTE* data_ptr,
                                             datastructure::ApplicationInputs& inputs);
  void setLinearVelocity1InApplicationInputs(const BYTE* data_ptr,
                                             datastructure::ApplicationInputs& inputs);
  void setLinearVelocityFlagsInApplicationInputs(const BYTE* data_ptr,
                                                 datastructure::ApplicationInputs& inputs);
  void setSleepModeInputInApplicationInputs(const BYTE* data_ptr,
                                            datastructure::ApplicationInputs& inputs);
  void setEvalutaionPathsOutputsInApplicationOutputs(const BYTE* data_ptr,
                                                     datastructure::ApplicationOutputs& outputs);
  void
  setEvaluationPathsOutputsEvalOutInApplicationOutputs(const BYTE* data_ptr,
                                                       datastructure::ApplicationOutputs& outputs);
  void
  setEvaluationPathsOutputsIsSafeInApplicationOutputs(const BYTE* data_ptr,
                                                      datastructure::ApplicationOutputs& outputs);
  void setEvaluationPathsOutputsValidFlagsInApplicationOutputs(
    const BYTE* data_ptr, datastructure::ApplicationOutputs& outputs);
  void setMonitoringCaseOutputsInApplicationOutputs(const BYTE* data_ptr,
                                                    datastructure::ApplicationOutputs& outputs);
  void setMonitoringCaseNumbersInApplicationOutputs(const BYTE* data_ptr,
                                                    datastructure::ApplicationOutputs& outputs);
  void setMonitoringCaseFlagsInApplicationOutputs(const BYTE* data_ptr,
                                                  datastructure::ApplicationOutputs& outputs);
  void setSleepModeOutputInApplicationOutputs(const BYTE* data_ptr,
                                              datastructure::ApplicationOutputs& outputs);
  void setErrorFlagsInApplicationOutputs(const BYTE* data_ptr,
                                         datastructure::ApplicationOutputs& outputs);
  void setLinearVelocityOutoutsInApplicationOutputs(const BYTE* data_ptr,
                                                    datastructure::ApplicationOutputs& outputs);
  void setLinearVelocity0InApplicationOutputs(const BYTE* data_ptr,
                                              datastructure::ApplicationOutputs& outputs);
  void setLinearVelocity1InApplicationOutputs(const BYTE* data_ptr,
                                              datastructure::ApplicationOutputs& outputs);
  void setLinearVelocityFlagsInApplicationOutputs(const BYTE* data_ptr,
                                                  datastructure::ApplicationOutputs& outputs);
  void setResultingVelocityOutputsInApplicationOutputs(const BYTE* data_ptr,
                                                       datastructure::ApplicationOutputs& outputs);
  void setResultingVelocityInApplicationOutputs(const BYTE* data_ptr,
                                                datastructure::ApplicationOutputs& outputs);
  void setResultingVelocityFlagsInApplicationOutputs(const BYTE* data_ptr,
                                                     datastructure::ApplicationOutputs& outputs);
  void setOutputFlagsinApplicationOutput(const BYTE* data_ptr,
                                         datastructure::ApplicationOutputs& outputs);
  bool checkIfPreconditionsAreMet(datastructure::Data& data);
  bool checkIfApplicationDataIsPublished(datastructure::Data& data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data& data);
};

} // namespace data_processing
} // namespace sick
