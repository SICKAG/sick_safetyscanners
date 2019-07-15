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
 * \file PacketBuffer.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include "sick_safetyscanners/datastructure/PacketBuffer.h"

namespace sick {
namespace datastructure {


PacketBuffer::PacketBuffer() {}

PacketBuffer::PacketBuffer(const std::vector<uint8_t>& buffer)
{
  setBuffer(buffer);
}

PacketBuffer::PacketBuffer(const PacketBuffer::ArrayBuffer& buffer, const size_t& length)
{
  setBuffer(buffer, length);
}

std::shared_ptr<std::vector<uint8_t> const> PacketBuffer::getBuffer() const
{
  // Okay to share since it's a shared_ptr<vector const>
  return m_buffer;
}

void PacketBuffer::setBuffer(const std::vector<uint8_t>& buffer)
{
  m_buffer = std::make_shared<std::vector<uint8_t> const>(buffer);
}

void PacketBuffer::setBuffer(const PacketBuffer::ArrayBuffer& buffer, const size_t& length)
{
  m_buffer = std::make_shared<std::vector<uint8_t> const>(buffer.data(), buffer.data() + length);
}

size_t PacketBuffer::getLength() const
{
  return m_buffer->size();
}

} // namespace datastructure
} // namespace sick
