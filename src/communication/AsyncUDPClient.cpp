#include <sick_microscan3_ros_driver/communication/AsyncUDPClient.h>

namespace sick {
namespace communication {
	AsyncUDPClient::AsyncUDPClient(PacketHandler packet_handler, std::string host, unsigned short server_port, unsigned short local_port) :
		socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port)),
                m_ioWorkPtr(),
		service_thread(&AsyncUDPClient::run_service, this),
                m_packet_handler(packet_handler)
	{
          m_ioWorkPtr = boost::make_shared<boost::asio::io_service::work>(boost::ref(io_service));
          io_service.run();
          std::cout << "setup udpclient" << std::endl;
	}

	AsyncUDPClient::~AsyncUDPClient()
	{
		io_service.stop();
		service_thread.join();
	}

	void AsyncUDPClient::start_receive()
	{
//          std::cout << "start receive" << std::endl;
            
		socket.async_receive_from(boost::asio::buffer(recv_buffer), remote_endpoint,
			[this](boost::system::error_code ec, std::size_t bytes_recvd){ this->handle_receive(ec, bytes_recvd); });
	}

	void AsyncUDPClient::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
	{
  //        std::cout << "handle receive" << std::endl;
		if (!error)
		{
                     m_packet_handler();
//			std::string message(recv_buffer.data(), recv_buffer.data() + bytes_transferred);
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
		while (!io_service.stopped()) {
                        std::cout << "running" << std::endl;
			try {
				io_service.run();
			}
			catch (const std::exception& e) {
                          std::cout << "Client: network exception: " <<  e.what() << std::endl;
			}
			catch (...) {
                          std::cout << "unknown exeption" << std::endl;
			}
		}
	}
}
}
