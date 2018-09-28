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
 * \file ParseIntrusionData.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef PARSEINTRUSIONDATA_H
#define PARSEINTRUSIONDATA_H

#include <boost/make_shared.hpp>

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

class ParseIntrusionData
{
public:
  ParseIntrusionData();

  datastructure::IntrusionData parseUDPSequence(sick::datastructure::PacketBuffer buffer,
                                                datastructure::Data& data);

  UINT16 getNumScanPoints() const;
  void setNumScanPoints(const UINT16& num_scan_points);

private:
  UINT16 m_num_scan_points;

  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  void setDataInIntrusionData(const BYTE* data_ptr, datastructure::IntrusionData& intrusion_data);
  void setDataInIntrusionDatums(const BYTE* data_ptr,
                                std::vector<sick::datastructure::IntrusionDatum>& intrusion_datums);
  UINT16 setDataInIntrusionDatum(UINT16 offset,
                                 const BYTE* data_ptr,
                                 sick::datastructure::IntrusionDatum& datum);
  UINT16 setSizeInIntrusionDatum(UINT16 offset,
                                 const BYTE* data_ptr,
                                 sick::datastructure::IntrusionDatum& datum);
  UINT16 setFlagsInIntrusionDatum(UINT16 offset,
                                  const BYTE* data_ptr,
                                  sick::datastructure::IntrusionDatum& datum);
  bool checkIfPreconditionsAreMet(datastructure::Data& data);
  bool checkIfIntrusionDataIsPublished(datastructure::Data& data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data& data);
};

} // namespace data_processing
} // namespace sick

#endif
