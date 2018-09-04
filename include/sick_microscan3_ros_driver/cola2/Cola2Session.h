#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/communication/AsyncTCPClient.h>

#include <sick_microscan3_ros_driver/cola2/Command.h>


namespace sick {
namespace cola2 {

class Command;

class Cola2Session
{
public:
  typedef boost::shared_ptr<sick::cola2::Command> CommandPtr;

  Cola2Session();

  void executeCommand(CommandPtr command);

  UINT32 getSessionID() const;
  void setSessionID(const UINT32 &sessionID);

private:
  void processPacket(const sick::datastructure::PacketBuffer& packet);

  bool addCommand(UINT16 request_id, CommandPtr command);
  bool findCommand(UINT16 request_id, CommandPtr& command);
  bool removeCommand(UINT16 request_id);


  boost::shared_ptr<sick::communication::AsyncTCPClient> m_async_tcp_client;

  std::map<UINT16, CommandPtr> m_pending_commands;

  UINT32 m_sessionID;
  UINT16 m_lastRequestID;
};


}
}
