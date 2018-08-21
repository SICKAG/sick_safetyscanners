// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file Microscan3.cpp
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-08-21
*
*/
//----------------------------------------------------------------------

#include "sick_microscan3_ros_driver/Microscan3.h"

namespace sick {

Microscan3::Microscan3(PaketReceivedCallbackFunction newPaketReceivedCallbackFunction)
  : m_newPaketReceivedCallbackFunction(newPaketReceivedCallbackFunction)
{
  std::cout << "starting microscan" << std::endl;
  sick::communication::AsyncUDPClient async_udp_client(std::bind(&Microscan3::processUDPPaket, this), "192.168.1.10",2122, 6060);
}

Microscan3::~Microscan3()
{
}

void Microscan3::processUDPPaket()
{
  std::cout << "process UDP Paket" << std::endl;
  m_newPaketReceivedCallbackFunction();
}

} /* namespace */
