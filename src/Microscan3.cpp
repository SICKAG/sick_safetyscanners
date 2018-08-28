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
  m_io_service_ptr = boost::make_shared<boost::asio::io_service>();
  m_async_udp_client = boost::make_shared<sick::communication::AsyncUDPClient>(boost::bind(&Microscan3::processUDPPaket, this, _1),
                                                                               boost::ref(*m_io_service_ptr), "192.168.1.10",2122, 6060);

  m_paket_merger = boost::make_shared<sick::data_processing::UDPPaketMerger>();
  std::cout << "started Microscan" << std::endl;
}

Microscan3::~Microscan3()
{
  m_udp_client_thread_ptr.reset();
  
}

bool Microscan3::run()
{
  std::cout << "enter run" << std::endl;

  m_udp_client_thread_ptr.reset(new boost::thread(boost::bind(&Microscan3::UDPClientThread, this)));

  m_async_udp_client->run_service();

}

bool Microscan3::UDPClientThread()
{
   std::cout << "enter thread" << std::endl;


   m_io_work_ptr = boost::make_shared<boost::asio::io_service::work>(boost::ref(*m_io_service_ptr));

   m_io_service_ptr->run();
   std::cout << "exit thread" << std::endl;
}





void Microscan3::processUDPPaket(const sick::datastructure::PaketBuffer& buffer)
{
  //std::cout << "process UDP Paket" << buffer.getBuffer().at(4) <<  std::endl;
  //std::cout << "Client: " <<buffer.getLength() << std::endl;
  if(m_paket_merger->addUDPPaket(buffer)) {

    m_newPaketReceivedCallbackFunction();
  }
}

} /* namespace */
