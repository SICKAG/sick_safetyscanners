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
 * \file ParseDerivedValues.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseDerivedValues.h>

namespace sick {
namespace data_processing {

ParseDerivedValues::ParseDerivedValues() {}

datastructure::DerivedValues
ParseDerivedValues::parseUDPSequence(const datastructure::PacketBuffer& buffer,
                                     datastructure::Data& data) const
{
  datastructure::DerivedValues derived_values;


  if (!checkIfPreconditionsAreMet(data))
  {
    derived_values.setIsEmpty(true);
    return derived_values;
  }
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr =
    vec_ptr->begin() + data.getDataHeaderPtr()->getDerivedValuesBlockOffset();
  setDataInDerivedValues(data_ptr, derived_values);
  return derived_values;
}

bool ParseDerivedValues::checkIfPreconditionsAreMet(const datastructure::Data& data) const
{
  if (!checkIfDerivedValuesIsPublished(data))
  {
    return false;
  }
  if (!checkIfDataContainsNeededParsedBlocks(data))
  {
    return false;
  }

  return true;
}

bool ParseDerivedValues::checkIfDerivedValuesIsPublished(const datastructure::Data& data) const
{
  return !(data.getDataHeaderPtr()->getDerivedValuesBlockOffset() == 0 &&
           data.getDataHeaderPtr()->getDerivedValuesBlockSize() == 0);
}

bool ParseDerivedValues::checkIfDataContainsNeededParsedBlocks(
  const datastructure::Data& data) const
{
  return !(data.getDataHeaderPtr()->isEmpty());
}

void ParseDerivedValues::setDataInDerivedValues(std::vector<uint8_t>::const_iterator data_ptr,
                                                datastructure::DerivedValues& derived_values) const
{
  setMultiplicationFactorInDerivedValues(data_ptr, derived_values);
  setNumberOfBeamsInDerivedValues(data_ptr, derived_values);
  setScanTimeInDerivedValues(data_ptr, derived_values);
  setStartAngleInDerivedValues(data_ptr, derived_values);
  setAngularBeamResolutionInDerivedValues(data_ptr, derived_values);
  setInterbeamPeriodInDerivedValues(data_ptr, derived_values);
}

void ParseDerivedValues::setMultiplicationFactorInDerivedValues(
  std::vector<uint8_t>::const_iterator data_ptr, datastructure::DerivedValues& derived_values) const
{
  derived_values.setMultiplicationFactor(read_write_helper::readUint16LittleEndian(data_ptr + 0));
}

void ParseDerivedValues::setNumberOfBeamsInDerivedValues(
  std::vector<uint8_t>::const_iterator data_ptr, datastructure::DerivedValues& derived_values) const
{
  derived_values.setNumberOfBeams(read_write_helper::readUint16LittleEndian(data_ptr + 2));
}

void ParseDerivedValues::setScanTimeInDerivedValues(
  std::vector<uint8_t>::const_iterator data_ptr, datastructure::DerivedValues& derived_values) const
{
  derived_values.setScanTime(read_write_helper::readUint16LittleEndian(data_ptr + 4));
}

void ParseDerivedValues::setStartAngleInDerivedValues(
  std::vector<uint8_t>::const_iterator data_ptr, datastructure::DerivedValues& derived_values) const
{
  derived_values.setStartAngle(read_write_helper::readInt32LittleEndian(data_ptr + 8));
}

void ParseDerivedValues::setAngularBeamResolutionInDerivedValues(
  std::vector<uint8_t>::const_iterator data_ptr, datastructure::DerivedValues& derived_values) const
{
  derived_values.setAngularBeamResolution(read_write_helper::readInt32LittleEndian(data_ptr + 12));
}

void ParseDerivedValues::setInterbeamPeriodInDerivedValues(
  std::vector<uint8_t>::const_iterator data_ptr, datastructure::DerivedValues& derived_values) const
{
  derived_values.setInterbeamPeriod(read_write_helper::readUint32LittleEndian(data_ptr + 16));
}

} // namespace data_processing
} // namespace sick
