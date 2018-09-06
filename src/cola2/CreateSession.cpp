#include <sick_microscan3_ros_driver/cola2/CreateSession.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

CreateSession::CreateSession(Cola2Session &session)
  :Command(session, 0x4F, 0x58) //see cola2 manual 0x4F = O, 0x58 = X
{

}

void CreateSession::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 5);
  BYTE* dataPtr = telegram.data() + prevSize;
  UINT8 heartBeatTimeoutSeconds = 60;
  sick::data_processing::ReadWriteHelper::writeUINT8BE(dataPtr, heartBeatTimeoutSeconds);
  //  memwrite<UINT8>(dataPtr, heartBeatTimeoutSeconds);
  UINT32 clientID = 12345; // some random number
  sick::data_processing::ReadWriteHelper::writeUINT32BE(dataPtr,clientID);
//  memwrite<UINT32>(dataPtr, clientID);
}

bool CreateSession::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool CreateSession::processReply()
{
  std::cout << "Opening Session processin" << std::endl;
  // Packet is already parsed by base class at this point

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

