#include <sick_microscan3_ros_driver/cola2/CloseSession.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

CloseSession::CloseSession(Cola2Session &session)
  :Command(session, 0x43, 0x58) //see cola2 manual 0x4F = O, 0x58 = X
{

}

void CloseSession::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
}

bool CloseSession::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool CloseSession::processReply()
{
  std::cout << "Closing Session processin" << std::endl;
  // Packet is already parsed by base class at this point

  if (getCommandType() == 'C' && getCommandMode() == 'A')
  {
    m_session.setSessionID(getSessionID());
    std::cout << "Successfully closed Cola2 session with sessionID: "
            << std::hex << m_session.getSessionID() << std::endl;
    return true;
  }
  else
  {
    return false;
  }
}

}
}

