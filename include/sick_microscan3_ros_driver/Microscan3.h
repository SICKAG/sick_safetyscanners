// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file Microscan3.h
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-08-21
*
*/
#pragma once

#include <boost/function.hpp>

#include <iostream>

#include <sick_microscan3_ros_driver/communication/AsyncUDPClient.h>

namespace sick {

/*!
 * Class containing the algorithmic part of the package.
 */
class Microscan3
{
 public:

   /*
    *  Typedef for function which has to be passed to this class. This enables the use of
    *  functions from the calling class. In this case a ROS publisher for the data.
    */
    typedef boost::function<void ()> PaketReceivedCallbackFunction;
   /*!
   * Constructor.
   */
  Microscan3(PaketReceivedCallbackFunction newPaketReceivedCallbackFunction);

  /*!
   * Destructor.
   */
  virtual ~Microscan3();

 private:
  PaketReceivedCallbackFunction m_newPaketReceivedCallbackFunction;

  void processUDPPaket();
};

} /* namespace */
