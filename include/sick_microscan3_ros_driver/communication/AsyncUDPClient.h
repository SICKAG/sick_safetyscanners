#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <boost/cstdint.hpp>
#include <boost/function.hpp>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <thread>
#include <functional>

#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>


namespace sick {
namespace communication{
	class AsyncUDPClient {
	public:
                //typedef boost::function<void(datastructure::PaketBuffer)> PacketHandler;
                typedef boost::function<void(const sick::datastructure::PacketBuffer&)> PacketHandler;

                AsyncUDPClient(PacketHandler packet_handler, boost::asio::io_service& io_service, unsigned short local_port = 0);
		virtual ~AsyncUDPClient();

                void run_service();

	private:

                datastructure::PacketBuffer::ArrayBuffer m_recv_buffer;

                PacketHandler m_packet_handler;

                boost::shared_ptr<boost::asio::io_service::work> m_io_work_ptr;
		// Network send/receive stuff
                boost::asio::io_service& m_io_service;
                boost::shared_ptr<boost::asio::ip::udp::socket> m_socket_ptr;
                boost::asio::ip::udp::endpoint m_remote_endpoint;
                std::thread m_service_thread;

		void start_receive();
		void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);


		AsyncUDPClient(AsyncUDPClient&); // block default copy constructor

	};
}//namespace
}
