#include <sick_microscan3_ros_driver/communication/AsyncTCPClient.h>

namespace sick {
namespace communication {
AsyncTCPClient::AsyncTCPClient(PacketHandler packet_handler, boost::asio::io_service& io_service, boost::asio::ip::address_v4 server_ip,
                               unsigned short server_port)
  : m_io_work_ptr()
  , m_io_service(io_service)
  , m_packet_handler(packet_handler)
{
  // Keep io_service busy
  m_io_work_ptr = boost::make_shared<boost::asio::io_service::work>(boost::ref(m_io_service));
  try
  {
    m_socket_ptr = boost::make_shared<boost::asio::ip::tcp::socket>(boost::ref(m_io_service));
  }
  catch (std::exception& e)
  {
    std::cout << "Exception while creating socket: " << e.what() << std::endl;
  }
  m_remote_endpoint = boost::asio::ip::tcp::endpoint(server_ip, server_port);
  std::cout << "TCP client setup" << std::endl;
}

AsyncTCPClient::~AsyncTCPClient()
{
  m_socket_ptr->close();
}

void AsyncTCPClient::do_connect(){
  //async connect takes some time, TODO set mutex to wait for connection
  m_socket_ptr->async_connect(m_remote_endpoint, [this](boost::system::error_code ec)
  {
    std::cout << "TCP error code: " << ec.value() << std::endl;
  });
}

void AsyncTCPClient::doSendAndReceive(const sick::datastructure::PacketBuffer::VectorBuffer& sendBuffer)
{
  //TODO remove when connect is mutexed and only return on connection
  sleep(1);

  if (!m_socket_ptr)
  {
    return;
  }
  boost::asio::async_write(*m_socket_ptr, boost::asio::buffer(sendBuffer),
    [this](boost::system::error_code ec, std::size_t bytes_send)
  {
    this->handleSendAndReceive(ec, bytes_send);
  });

}

void AsyncTCPClient::initiateReceive()
{
  if (!m_socket_ptr)
  {
    return;
  }
  m_socket_ptr->async_read_some(boost::asio::buffer(m_recv_buffer),
    [this](boost::system::error_code ec, std::size_t bytes_recvd)
  {
    this->handle_receive(ec, bytes_recvd);
  });

}

void AsyncTCPClient::setPacketHandler(const PacketHandler &packet_handler)
{
  m_packet_handler = packet_handler;
}

void AsyncTCPClient::handleSendAndReceive(const boost::system::error_code& error,
                                          std::size_t bytes_transferred)
{
  // Check for errors
  if (!error || error == boost::asio::error::message_size)
  {
    initiateReceive();
  }
  else
  {
    std::cout << "Error in tcp handle send and receive: " << error.value() << std::endl;
  }
}


void AsyncTCPClient::start_receive()
{
}

void AsyncTCPClient::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (!error)
  {
    sick::datastructure::PacketBuffer paket_buffer(m_recv_buffer, bytes_transferred);
    m_packet_handler(paket_buffer);
  }
  else
  {
    std::cout << "Error in tcp handle receive: " << error.value() <<  std::endl;
  }
}


void AsyncTCPClient::run_service()
{
}

}
}
