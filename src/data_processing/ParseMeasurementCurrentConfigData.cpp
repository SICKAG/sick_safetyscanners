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
 * \file ParseMeasurementCurrentConfigData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseMeasurementCurrentConfigData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseMeasurementCurrentConfigData::ParseMeasurementCurrentConfigData() {}


bool ParseMeasurementCurrentConfigData::parseTCPSequence(
  const datastructure::PacketBuffer& buffer, datastructure::ConfigData& config_data) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vecPtr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr            = vecPtr->begin();
  config_data.setStartAngle(readStartAngle(data_ptr));
  config_data.setAngularBeamResolution(readAngularBeamResolution(data_ptr));
  return true;
}

uint32_t ParseMeasurementCurrentConfigData::readStartAngle(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return ReadWriteHelper::readuint32_tLittleEndian(data_ptr + 36);
}

uint32_t ParseMeasurementCurrentConfigData::readAngularBeamResolution(
  std::vector<uint8_t>::const_iterator data_ptr) const
{
  return ReadWriteHelper::readuint32_tLittleEndian(data_ptr + 40);
}

} // namespace data_processing
} // namespace sick
