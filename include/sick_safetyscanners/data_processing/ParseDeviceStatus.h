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
 * \file DeviceStatusVariableCommand.h
 *
 * \author  Jonathan Meyer <jmeyer1292@gmail.com>
 * \date    2019-03-11
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDEVICESTATUS_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDEVICESTATUS_H

#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>
#include <sick_safetyscanners/datastructure/DeviceStatus.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.h>

namespace sick {

namespace data_processing {

/*!
 * \brief Parser to read the device status from a tcp sequence.
 */
class ParseDeviceStatus
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseDeviceStatus();

  /*!
   * \brief Parses a tcp sequence to read the device status of the sensor.
   *
   * \param buffer The incoming tcp sequence.
   * \param type_code Reference to the device status, which will be written while parsing.
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::DeviceStatus& device_status) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
};

}
}

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEDEVICESTATUS_H
