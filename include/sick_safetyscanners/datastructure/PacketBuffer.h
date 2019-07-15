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
 * \file PacketBuffer.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_PACKETBUFFER_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_PACKETBUFFER_H

#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <sick_safetyscanners/datastructure/DatagramHeader.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>


namespace sick {
namespace datastructure {

/*!
 * \brief MAXSIZE of the packetbuffer
 */
const uint32_t MAXSIZE = 10000;

/*!
 * \brief A packetbuffer for the raw data from the sensor.
 */
class PacketBuffer
{
public:
  /*!
   * \brief Typedef for an arraybuffer which can be read from the sensor.
   */
  typedef boost::array<uint8_t, MAXSIZE> ArrayBuffer;

  /*!
   * \brief Constructor of PacketBuffer.
   */
  PacketBuffer();

  /*!
   * \brief Constructor of PacketBuffer.
   *
   * No explicit tag used to allow implicit conversion from different inputs types.
   *
   * \param buffer vector to set for the PacketBuffer.
   */
  PacketBuffer(const std::vector<uint8_t>& buffer);

  /*!
   * \brief Constructor of PacketBuffer.
   * \param buffer Arraybuffer to set for the PacketBuffer.
   * \param length Length of the array.
   */
  PacketBuffer(const ArrayBuffer& buffer, const size_t& length);

  /*!
   * \brief Returns defined maximum size of PacketBuffer.
   * \return Maximum size of PacketBuffer.
   */
  static uint32_t getMaxSize() { return MAXSIZE; }

  /*!
   * \brief Getter to return a copy of the data saved in the PacketBuffer.
   * \return Data copy as shared_ptr<vector const>.  Needed because this bufer could be overwritten.
   */
  std::shared_ptr<std::vector<uint8_t> const> getBuffer() const;

  /*!
   * \brief Setter for the PacketBuffer.
   * \param buffer Input VectorBuffer to save.
   */
  void setBuffer(const std::vector<uint8_t>& buffer);

  /*!
   * \brief Setter for the PacketBuffer.
   * \param buffer Input ArrayBuffer to save.
   * \param length Length of input ArrayBuffer.
   */
  void setBuffer(const ArrayBuffer& buffer, const size_t& length);

  /*!
   * \brief Returns length of the current PacketBuffer.
   * \return Length.
   */
  size_t getLength() const;


private:
  std::shared_ptr<std::vector<uint8_t> const> m_buffer;
};

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_PACKETBUFFER_H
