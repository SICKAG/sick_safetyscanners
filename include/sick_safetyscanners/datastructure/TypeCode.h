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
 * \file TypeCode.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_TYPECODE_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_TYPECODE_H

#include <iostream>


namespace sick {
namespace datastructure {

enum e_interface_type
{
  E_EFIPRO,
  E_ETHERNET_IP,
  E_PROFINET         = 3,
  E_NONSAFE_ETHERNET = 4
};

enum e_ranges
{
  E_NORMAL_RANGE = 40,
  E_LONG_RANGE   = 64
};


/*!
 * \brief Class containing the type code of a laser scanner.
 */
class TypeCode
{
public:
  /*!
   * \brief Constructor of the type code.
   */
  TypeCode();

  /*!
   * \brief Gets the type code for the scanner.
   *
   * \returns The type code for the scanner.
   */
  std::string getTypeCode() const;
  /*!
   * \brief Sets the type code for the scanner.
   *
   * \param type_code The type code for the scanner.
   */
  void setTypeCode(const std::string& type_code);
  /*!
   * \brief Gets the interface type for the scanner.
   *
   * \returns The interface type for the scanner.
   */
  uint8_t getInterfaceType() const;
  /*!
   * \brief Sets the interface type for the scanner.
   *
   * \param interface_type The interface type for the scanner.
   */
  void setInterfaceType(const uint8_t& interface_type);

  /*!
   * \brief Gets the max range for the scanner.
   *
   * \returns The max range for the scanner.
   */
  float getMaxRange() const;
  /*!
   * \brief Sets the max range for the scanner.
   *
   * \param max_distance The max range for the scanner.
   */
  void setMaxRange(const float& max_distance);

private:
  std::string m_type_code;
  uint8_t m_interface_type;
  float m_max_range;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_TYPECODE_H
