#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/communication/AsyncTCPClient.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>
#include <sick_microscan3_ros_driver/cola2/CreateSession.h>
#include <sick_microscan3_ros_driver/cola2/CloseSession.h>

#include <sick_microscan3_ros_driver/data_processing/ParseTCPPacket.h>
#include <sick_microscan3_ros_driver/data_processing/TCPPacketMerger.h>

#include <boost/bind.hpp>

namespace sick {
namespace cola2 {

class Command;
class CreateSession;

class Cola2Session
{
public:
  typedef boost::shared_ptr<sick::cola2::Command> CommandPtr;

  Cola2Session(boost::shared_ptr<communication::AsyncTCPClient> async_tcp_client);

  bool executeCommand(CommandPtr command);

  UINT32 getSessionID() const;
  void setSessionID(const UINT32 &sessionID);

  UINT16 getNextRequestID();

  bool close();
private:
  void processPacket(const sick::datastructure::PacketBuffer& packet);

  bool addCommand(UINT16 request_id, CommandPtr command);
  bool findCommand(UINT16 request_id, CommandPtr& command);
  bool removeCommand(UINT16 request_id);


  boost::shared_ptr<sick::communication::AsyncTCPClient> m_async_tcp_client;
  boost::shared_ptr<sick::data_processing::ParseTCPPacket> m_parser;
  boost::shared_ptr<sick::data_processing::TCPPaketMerger> m_packetMerger;

  std::map<UINT16, CommandPtr> m_pending_commands;

  UINT32 m_sessionID;
  UINT16 m_lastRequestID;
  bool open();
};


}
}
