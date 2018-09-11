#include <sick_microscan3_ros_driver/cola2/CreateSession.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

CreateSession::CreateSession(Cola2Session &session)
  :Command(session, 0x4F, 0x58) //see cola2 manual 0x4F = O, 0x58 = X
{
  m_writer_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();


}

void CreateSession::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  BYTE* data_ptr = prepareTelegramAndGetDataPtr(telegram);
  writeHeartbeatTimeoutToDataPtr(data_ptr);
  writeClientIdToDataPtr(data_ptr);
}

BYTE* CreateSession::prepareTelegramAndGetDataPtr(sick::datastructure::PacketBuffer::VectorBuffer& telegram)  const
{
  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 5);
  return telegram.data() + prevSize;
}

bool CreateSession::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool CreateSession::processReply()
{
  if (getCommandType() == 'O' && getCommandMode() == 'A')
  {
    m_session.setSessionID(getSessionID());
    std::cout << "Successfully opened Cola2 session with sessionID: "
            << std::hex << m_session.getSessionID() << std::endl;
    return true;
  }
  else
  {
    std::cout << "Could not open Cola2 session"<< std::endl;
    return false;
  }
}

bool CreateSession::writeHeartbeatTimeoutToDataPtr(BYTE*& data_ptr) const
{
  UINT8 heartBeatTimeoutSeconds = 60;
  m_writer_ptr->writeUINT8BigEndian(data_ptr, heartBeatTimeoutSeconds, 0);
}

bool CreateSession::writeClientIdToDataPtr(BYTE*& data_ptr) const
{
  UINT32 clientID = 1; // some random number
  m_writer_ptr->writeUINT32BigEndian(data_ptr,clientID,1);
}

}
}

