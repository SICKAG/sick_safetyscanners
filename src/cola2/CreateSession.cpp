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
  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 5);
  BYTE* data_ptr = telegram.data() + prevSize;
  UINT8 heartBeatTimeoutSeconds = 60;
  m_writer_ptr->writeUINT8BigEndian(data_ptr, heartBeatTimeoutSeconds);
  //  memwrite<UINT8>(data_ptr, heartBeatTimeoutSeconds);
  UINT32 clientID = 12345; // some random number
  m_writer_ptr->writeUINT32BigEndian(data_ptr,clientID);
//  memwrite<UINT32>(data_ptr, clientID);
}

bool CreateSession::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool CreateSession::processReply()
{
  std::cout << "Opening Session processin" << std::endl;
  // Packet is already parsed by base class at this point

  std::cout << "Command: " << getCommandType() <<  "," << getCommandMode() << std::endl;

  if (getCommandType() == 'O' && getCommandMode() == 'A')
  {
    m_session.setSessionID(getSessionID());
    std::cout << "Successfully opened Cola2 session with sessionID: "
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

