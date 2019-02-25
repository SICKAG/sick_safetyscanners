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
 * \file ParseMeasurementCurrentConfigData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMEASUREMENTCURRENTCONFIGDATA_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMEASUREMENTCURRENTCONFIGDATA_H

#include <sick_safetyscanners/datastructure/ConfigData.h>
#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.h>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read the field header for protective and warning fields.
 */
class ParseMeasurementCurrentConfigData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseMeasurementCurrentConfigData();

  /*!
   * \brief Parses a tcp sequence to read the header for a warning or protective field.
   *
   * \param buffer The incoming tcp sequence.
   * \param config_data Reference to the configuration data where the information will be set.
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::ConfigData& config_data) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  uint32_t readStartAngle(const uint8_t* data_ptr) const;
  uint32_t readAngularBeamResolution(const uint8_t* data_ptr) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMEASUREMENTCURRENTCONFIGDATA_H
