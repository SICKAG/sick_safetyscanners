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
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <iostream>

#include <sick_microscan3_ros_driver/communication/AsyncUDPClient.h>
#include <sick_microscan3_ros_driver/communication/AsyncTCPClient.h>
#include <sick_microscan3_ros_driver/data_processing/UDPPacketMerger.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/data_processing/ParseData.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/ChangeCommSettingsCommand.h>

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
    typedef boost::function<void (const sick::datastructure::Data&)> PaketReceivedCallbackFunction;
   /*!
   * Constructor.
   */
  Microscan3(PaketReceivedCallbackFunction newPaketReceivedCallbackFunction);

  /*!
   * Destructor.
   */
  virtual ~Microscan3();

    bool run();

    void serviceTCP();
private:
  PaketReceivedCallbackFunction m_newPaketReceivedCallbackFunction;

  boost::shared_ptr<boost::asio::io_service> m_io_service_ptr;
  boost::shared_ptr<boost::asio::io_service::work> m_io_work_ptr;
  boost::shared_ptr<sick::communication::AsyncUDPClient> m_async_udp_client_ptr;
  boost::shared_ptr<sick::communication::AsyncTCPClient> m_async_tcp_client_ptr;
  boost::scoped_ptr<boost::thread> m_udp_client_thread_ptr;

  boost::shared_ptr<sick::cola2::Cola2Session> m_session_ptr;

  boost::shared_ptr<sick::data_processing::UDPPaketMerger> m_paket_merger_ptr;

  void processUDPPaket(const datastructure::PacketBuffer &buffer);
  bool UDPClientThread();
  void processTCPPaket(const sick::datastructure::PacketBuffer &buffer);
  void startTCPConnection();
  void changeCommSettingsinColaSession();
  void stopTCPConnection();
};

} /* namespace */
