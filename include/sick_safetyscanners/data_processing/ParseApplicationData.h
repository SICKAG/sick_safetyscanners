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

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEAPPLICATIONDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEAPPLICATIONDATA_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/DerivedValues.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

#include <vector>


namespace sick {
namespace data_processing {

/*!
 * \brief Parses the application data from a packet buffer.
 */
class ParseApplicationData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseApplicationData();

  /*!
   * \brief Parses the application data from the packet buffer.
   *
   * \param buffer The incoming packet buffer with raw data.
   * \param data The already parsed data, used to check if needed data is there.
   *
   * \returns The parsed application data.
   */
  datastructure::ApplicationData parseUDPSequence(const sick::datastructure::PacketBuffer& buffer,
                                                  datastructure::Data& data) const;

private:
  void setDataInApplicationData(std::vector<uint8_t>::const_iterator data_ptr,
                                datastructure::ApplicationData& application_data) const;
  void
  setApplicationInputsInApplicationData(std::vector<uint8_t>::const_iterator data_ptr,
                                        datastructure::ApplicationData& application_data) const;
  void
  setApplicationOutputsInApplicationData(std::vector<uint8_t>::const_iterator data_ptr,
                                         datastructure::ApplicationData& application_data) const;
  void setDataInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                  datastructure::ApplicationInputs& inputs) const;
  void setDataInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                   datastructure::ApplicationOutputs& outputs) const;
  void setUnsafeInputsInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                          datastructure::ApplicationInputs& inputs) const;
  void setUnsafeInputsSourcesInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                 datastructure::ApplicationInputs& inputs) const;
  void setUnsafeInputsFlagsInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                               datastructure::ApplicationInputs& inputs) const;
  void setMonitoringCaseInputsInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                  datastructure::ApplicationInputs& inputs) const;
  void setMonitoringCaseNumbersInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                   datastructure::ApplicationInputs& inputs) const;
  void setMonitoringCaseFlagsInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                 datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocityInputsInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                  datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocity0InApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                             datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocity1InApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                             datastructure::ApplicationInputs& inputs) const;
  void setLinearVelocityFlagsInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                 datastructure::ApplicationInputs& inputs) const;
  void setSleepModeInputInApplicationInputs(std::vector<uint8_t>::const_iterator data_ptr,
                                            datastructure::ApplicationInputs& inputs) const;
  void
  setEvalutaionPathsOutputsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                datastructure::ApplicationOutputs& outputs) const;
  void setEvaluationPathsOutputsEvalOutInApplicationOutputs(
    std::vector<uint8_t>::const_iterator data_ptr,
    datastructure::ApplicationOutputs& outputs) const;
  void setEvaluationPathsOutputsIsSafeInApplicationOutputs(
    std::vector<uint8_t>::const_iterator data_ptr,
    datastructure::ApplicationOutputs& outputs) const;
  void setEvaluationPathsOutputsValidFlagsInApplicationOutputs(
    std::vector<uint8_t>::const_iterator data_ptr,
    datastructure::ApplicationOutputs& outputs) const;
  void
  setMonitoringCaseOutputsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                               datastructure::ApplicationOutputs& outputs) const;
  void
  setMonitoringCaseNumbersInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                               datastructure::ApplicationOutputs& outputs) const;
  void setMonitoringCaseFlagsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                  datastructure::ApplicationOutputs& outputs) const;
  void setSleepModeOutputInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                              datastructure::ApplicationOutputs& outputs) const;
  void setErrorFlagsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                         datastructure::ApplicationOutputs& outputs) const;
  void
  setLinearVelocityOutoutsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                               datastructure::ApplicationOutputs& outputs) const;
  void setLinearVelocity0InApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                              datastructure::ApplicationOutputs& outputs) const;
  void setLinearVelocity1InApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                              datastructure::ApplicationOutputs& outputs) const;
  void setLinearVelocityFlagsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                  datastructure::ApplicationOutputs& outputs) const;
  void
  setResultingVelocityOutputsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                  datastructure::ApplicationOutputs& outputs) const;
  void setResultingVelocityInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                datastructure::ApplicationOutputs& outputs) const;
  void
  setResultingVelocityFlagsInApplicationOutputs(std::vector<uint8_t>::const_iterator data_ptr,
                                                datastructure::ApplicationOutputs& outputs) const;
  void setOutputFlagsinApplicationOutput(std::vector<uint8_t>::const_iterator data_ptr,
                                         datastructure::ApplicationOutputs& outputs) const;
  bool checkIfPreconditionsAreMet(const datastructure::Data& data) const;
  bool checkIfApplicationDataIsPublished(const datastructure::Data& data) const;
  bool checkIfDataContainsNeededParsedBlocks(const datastructure::Data& data) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEAPPLICATIONDATA_H
