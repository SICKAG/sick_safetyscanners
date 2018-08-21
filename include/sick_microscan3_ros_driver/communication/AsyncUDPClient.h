#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <boost/cstdint.hpp>
#include <boost/function.hpp>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <thread>


namespace sick {
namespace communication{
	class AsyncUDPClient {
	public:
                typedef boost::uint8_t                   BYTE;         ///<  8 bit wide unsigned integer 
                typedef BYTE array_type;
                typedef boost::array<array_type, 10000> ArrayBuffer;

                typedef boost::function<void()> PacketHandler;

                AsyncUDPClient(PacketHandler packet_handler, std::string host, unsigned short server_port, unsigned short local_port = 0);
		virtual ~AsyncUDPClient();

	private:

                ArrayBuffer recv_buffer;

                PacketHandler m_packet_handler;

                boost::shared_ptr<boost::asio::io_service::work> m_ioWorkPtr;
		// Network send/receive stuff
                boost::asio::io_service io_service;
                boost::asio::ip::udp::socket socket;
                boost::asio::ip::udp::endpoint remote_endpoint;
                std::thread service_thread;

		void start_receive();
		void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
		void run_service();

		AsyncUDPClient(AsyncUDPClient&); // block default copy constructor

	};
}//namespace
}
