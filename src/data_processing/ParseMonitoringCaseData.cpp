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
 * \file ParseMonitoringCaseData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-11-29
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ParseMonitoringCaseData.h>

#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseMonitoringCaseData::ParseMonitoringCaseData()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}


bool ParseMonitoringCaseData::parseTCPSequence(
  const datastructure::PacketBuffer& buffer,
  sick::datastructure::MonitoringCaseData& monitoring_case_data) const
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  bool valid = isValid(data_ptr);
  monitoring_case_data.setIsValid(valid);
  return true;
}

bool ParseMonitoringCaseData::isValid(const uint8_t*& data_ptr) const
{
  bool res     = false;
  uint8_t byte = m_reader_ptr->readuint8_t(data_ptr, 0);
  if (byte == 'R' || byte == 'Y')
  {
    res = true;
  }
  return res;
}

uint16_t ParseMonitoringCaseData::readMonitoringCaseNumber(const uint8_t*& data_ptr) const
{
  return m_reader_ptr->readuint16_tLittleEndian(data_ptr, 6);
}

} // namespace data_processing
} // namespace sick
