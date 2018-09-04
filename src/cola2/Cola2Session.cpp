#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>

namespace sick {
namespace cola2 {

Cola2Session::Cola2Session()
{

}

void Cola2Session::executeCommand(CommandPtr command)
{

}

UINT32 Cola2Session::getSessionID() const
{
  return m_sessionID;
}

void Cola2Session::setSessionID(const UINT32 &sessionID)
{
  m_sessionID = sessionID;
}

void Cola2Session::processPacket(const datastructure::PacketBuffer &packet)
{
  std::cout << "Processing TCP packet in Session" << std::endl;
}

bool Cola2Session::addCommand(UINT16 request_id, CommandPtr command)
{
  if(m_pending_commands.find(request_id) != m_pending_commands.end())
  {
    return false;
  }
  m_pending_commands[request_id] = command;
  return true;

}

bool Cola2Session::findCommand(UINT16 request_id, CommandPtr &command)
{
  if (m_pending_commands.find(request_id) == m_pending_commands.end())
  {
    return false;
  }
  command = m_pending_commands[request_id];
  return true;
}

bool Cola2Session::removeCommand(UINT16 request_id)
{
  auto it = m_pending_commands.find(request_id);
  if(it == m_pending_commands.end())
  {
    return false;
  }

  m_pending_commands.erase(it);
  return true;

}

}
}
