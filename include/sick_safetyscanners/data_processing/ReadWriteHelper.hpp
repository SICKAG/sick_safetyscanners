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
 * \file ReadWriteHelper.hpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_READWRITEHELPER_HPP
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_READWRITEHELPER_HPP

#include <stdint.h>

namespace sick {
namespace ReadWriteHelper {

  /*!
   * \brief Helper Functions read and write data at a certain place in a buffer.
   */

  /*!
   * \brief Writes an unsigned 8-bit integer to a buffer at offset.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeuint8_t(uint8_t*& buf, const uint8_t v, const uint16_t offset)
  {
    buf[offset] = v;
  }

  /*!
   * \brief Writes an unsigned 8-bit integer to a buffer at offset in big endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeuint8_tBigEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset)
  {
    writeuint8_t(buf, v, offset);
  }

  /*!
   * \brief Writes an unsigned 8-bit integer to a buffer at offset in little endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeuint8_tLittleEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset)
  {
    writeuint8_t(buf, v, offset);
  }

  /*!
   * \brief Writes a signed 8-bit integer to a buffer at offset.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeint8_t(uint8_t*& buf, const uint8_t v, const uint16_t offset)
  {
    writeuint8_t(buf, v, offset);
  }

  /*!
   * \brief Writes a signed 8-bit integer to a buffer at offset in big endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeint8_tBigEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset)
  {
    writeint8_t(buf, v, offset);
  }

  /*!
   * \brief Writes a signed 8-bit integer to a buffer at offset in little endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeint8_tLittleEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset)
  {
    writeint8_t(buf, v, offset);
  }

  /*!
   * \brief Writes an unsigned 16-bit integer to a buffer at offset in big endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeuint16_tBigEndian(uint8_t*& buf, const uint16_t v, const uint16_t offset)
  {
    buf[offset]     = (v & 0xff00) >> 8;
    buf[offset + 1] = v & 0xff;
  }

  /*!
   * \brief Writes an unsigned 16-bit integer to a buffer at offset in little endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeuint16_tLittleEndian(uint8_t*& buf, const uint16_t v, const uint16_t offset)
  {
    buf[offset + 1] = (v & 0xff00) >> 8;
    buf[offset]     = v & 0xff;
  }


  /*!
   * \brief Writes an unsigned 32-bit integer to a buffer at offset in big endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeuint32_tBigEndian(uint8_t*& buf, const uint32_t v, const uint16_t offset)
  {
    buf[offset]     = (v & 0xff000000) >> 24;
    buf[offset + 1] = (v & 0xff0000) >> 16;
    buf[offset + 2] = (v & 0xff00) >> 8;
    buf[offset + 3] = v & 0xff;
  }

  /*!
   * \brief Writes an unsigned 32-bit integer to a buffer at offset in little endian encoding.
   *
   * \param buf The buffer to write to.
   * \param v Value which will be written.
   * \param offset Position the value will be written to.
   */
  inline void writeuint32_tLittleEndian(uint8_t*& buf, const uint32_t v, const uint16_t offset)
  {
    buf[offset + 3] = (v & 0xff000000) >> 24;
    buf[offset + 2] = (v & 0xff0000) >> 16;
    buf[offset + 1] = (v & 0xff00) >> 8;
    buf[offset]     = v & 0xff;
  }

  /*!
   * \brief Read an unsigned 8-bit integer at offset.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline uint8_t readuint8_t(const uint8_t*& buf, const uint16_t offset)
  {
    uint8_t value = buf[offset];
    return value;
  }

  /*!
   * \brief Read an unsigned 8-bit integer at offset in big endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline uint8_t readuint8_tBigEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readuint8_t(buf, offset);
  }

  /*!
   * \brief Read an unsigned 8-bit integer at offset in big little encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline uint8_t readuint8_tLittleEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readuint8_t(buf, offset);
  }

  /*!
   * \brief Read a signed 8-bit integer at offset.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline int8_t readint8_t(const uint8_t*& buffer, const uint16_t offset)
  {
    return readuint8_t(buffer, offset);
  }

  /*!
   * \brief Read a signed 8-bit integer at offset in big endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline int8_t readint8_tBigEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readint8_t(buf, offset);
  }

  /*!
   * \brief Read a signed 8-bit integer at offset in little endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline int8_t readint8_tLittleEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readint8_t(buf, offset);
  }

  /*!
   * \brief Read an unsigned 16-bit integer at offset in big endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline uint16_t readuint16_tBigEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return (buf[offset] << 8) + buf[offset + 1];
  }

  /*!
   * \brief Read an unsigned 16-bit integer at offset in little endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline uint16_t readuint16_tLittleEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return (buf[offset + 1] << 8) + buf[offset];
  }

  /*!
   * \brief Read a signed 16-bit integer at offset in big endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline int16_t readint16_tBigEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readuint16_tBigEndian(buf, offset);
  }

  /*!
   * \brief Read a signed 16-bit integer at offset in little endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline int16_t readint16_tLittleEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readuint16_tLittleEndian(buf, offset);
  }

  /*!
   * \brief Read an unsigned 32-bit integer at offset in big endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline uint32_t readuint32_tBigEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return (buf[offset] << 24) + (buf[offset + 1] << 16) + (buf[offset + 2] << 8) + buf[offset + 3];
  }

  /*!
   * \brief Read an unsigned 32-bit integer at offset in little endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline uint32_t readuint32_tLittleEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return (buf[offset + 3] << 24) + (buf[offset + 2] << 16) + (buf[offset + 1] << 8) + buf[offset];
  }

  /*!
   * \brief Read an unsigned 32-bit integer at offset in big endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline int32_t readint32_tBigEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readuint32_tBigEndian(buf, offset);
  }


  /*!
   * \brief Read an unsigned 32-bit integer at offset in little endian encoding.
   *
   * \param buf Buffer to read from.
   * \param offset Position of integer.
   *
   * \returns The value of the read integer.
   */
  inline int32_t readint32_tLittleEndian(const uint8_t*& buf, const uint16_t offset)
  {
    return readuint32_tLittleEndian(buf, offset);
  }

} // namespace ReadWriteHelper
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_READWRITEHELPER_HPP
