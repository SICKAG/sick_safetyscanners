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
* \file ParseDerivedValues.h
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

class ParseDerivedValues : public AbstractParseUDPSequence
{
public:
  ParseDerivedValues();

  datastructure::DerivedValues parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setMultiplicationFactorInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setNumberOfBeamsInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setScanTimeInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setStartAngleInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setAngularBeamResolutionInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setInterbeamPeriodInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool checkIfPreconditionsAreMet(datastructure::Data &data);
  bool checkIfDerivedValuesIsPublished(datastructure::Data &data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data &data);
};

}
}


