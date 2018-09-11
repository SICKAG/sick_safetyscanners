#include <sick_microscan3_ros_driver/communication/AsyncUDPClient.h>

namespace sick {
namespace communication {
  AsyncUDPClient::AsyncUDPClient(PacketHandler packet_handler, boost::asio::io_service& io_service, std::string host, unsigned short server_port, unsigned short local_port) :
    //socket(boost::ref(io_service), boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port)),
                m_io_work_ptr(),
                m_io_service(io_service),
                m_packet_handler(packet_handler)
	{
          // Keep io_service busy so that call to io_service-> run does not return imediatly
          m_io_work_ptr = boost::make_shared<boost::asio::io_service::work>(boost::ref(m_io_service));
          try
          {

            // open the socket and bind it to the port specified by m_portToReceiveAt
            // boost::ref must be used to be able to pass io_service by reference
            m_socket_ptr = boost::make_shared<boost::asio::ip::udp::socket>(boost::ref(m_io_service),
                    boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                        local_port));
          }
          catch (std::exception& e)
          {
            std::cout << "Exception while creating socket: " << e.what() << std::endl;
          }
          std::cout << "setup udpclient" << std::endl;
	}

	AsyncUDPClient::~AsyncUDPClient()
	{
    m_io_service.stop();
  }

	void AsyncUDPClient::start_receive()
	{
          std::cout << "start receive" << std::endl;

          m_socket_ptr->async_receive_from(boost::asio::buffer(m_recv_buffer), m_remote_endpoint,
      [this](boost::system::error_code ec, std::size_t bytes_recvd){ this->handle_receive(ec, bytes_recvd); });
	}

	void AsyncUDPClient::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
	{
    std::cout << "handle receive" << std::endl;
		if (!error)
		{
      sick::datastructure::PacketBuffer paket_buffer(m_recv_buffer, bytes_transferred);
      //std::cout << m_recv_buffer.size() << std::endl;
      //int paket_buffer = 3;
      //std::cout << "Client: " <<paket_buffer.getLength() << std::endl;
      m_packet_handler(paket_buffer);
		}
		else
		{
      std::cout << "error in handle receive" << std::endl;
		}

    start_receive();
	}


	void AsyncUDPClient::run_service()
	{
    std::cout << "runservice" << std::endl;
    start_receive();
	}

}
}
