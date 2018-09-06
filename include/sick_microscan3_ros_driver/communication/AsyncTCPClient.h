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

//TODO refactor
namespace sick {
namespace communication{
        class AsyncTCPClient {
	public:
                //typedef boost::function<void(datastructure::PaketBuffer)> PacketHandler;
                typedef boost::function<void(const sick::datastructure::PacketBuffer&)> PacketHandler;

                AsyncTCPClient(PacketHandler packet_handler, boost::asio::io_service& io_service, std::string host,
                               unsigned short server_port, unsigned short local_port = 0);
                virtual ~AsyncTCPClient();

                void run_service();

                void do_connect();
                void doSendAndReceive(const  sick::datastructure::PacketBuffer::VectorBuffer &sendBuffer);
                void initiateReceive();
                void setPacketHandler(const PacketHandler &packet_handler);

        private:

                datastructure::PacketBuffer::ArrayBuffer m_recv_buffer;

                PacketHandler m_packet_handler;

                boost::shared_ptr<boost::asio::io_service::work> m_ioWorkPtr;
		// Network send/receive stuff
                boost::asio::io_service& m_io_service;
                boost::shared_ptr<boost::asio::ip::tcp::socket> m_socket;
                boost::asio::ip::tcp::endpoint m_remote_endpoint;
                std::thread m_service_thread;

		void start_receive();
		void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);


                AsyncTCPClient(AsyncTCPClient&); // block default copy constructor

                void handleSendAndReceive(const boost::system::error_code &error, std::size_t);
        };
}//namespace
}
