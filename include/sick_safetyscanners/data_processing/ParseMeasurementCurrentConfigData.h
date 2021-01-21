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

#include <boost/asio/ip/address_v4.hpp>
#include <sick_safetyscanners/datastructure/ConfigData.h>
#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read the current configuration of the sensor.
 */
class ParseMeasurementCurrentConfigData
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseMeasurementCurrentConfigData();

  /*!
   * \brief Parses a tcp sequence to read the current configuration of the sensor.
   *
   * \param buffer The incoming tcp sequence.
   * \param config_data Reference to the config data where the current configuration will be set.
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::ConfigData& config_data) const;

private:
  std::string readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const;
  bool readEnabled(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint8_t readInterfaceType(std::vector<uint8_t>::const_iterator data_ptr) const;
  boost::asio::ip::address_v4 readHostIp(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readHostPort(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readPublishingFreq(std::vector<uint8_t>::const_iterator data_ptr) const;
  int32_t readEndAngle(std::vector<uint8_t>::const_iterator data_ptr) const;
  int32_t readStartAngle(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readFeatures(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readDerivedMultiplicationFactor(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readDerivedNumBeams(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint16_t readDerivedScanTime(std::vector<uint8_t>::const_iterator data_ptr) const;
  int32_t readDerivedStartAngle(std::vector<uint8_t>::const_iterator data_ptr) const;
  int32_t readDerivedAngularBeamResolution(std::vector<uint8_t>::const_iterator data_ptr) const;
  uint32_t readDerivedInterbeamPeriod(std::vector<uint8_t>::const_iterator data_ptr) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_PARSEMEASUREMENTCURRENTCONFIGDATA_H
