#include <sick_microscan3_ros_driver/cola2/MethodCommand.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

MethodCommand::MethodCommand(Cola2Session &session, UINT16 method_index)
  :Command(session,0x4D, 0x49) // see cola2 manual 0x4D = 'M' and  0x49 = 'I'
  , m_method_index(method_index)
{

}

void MethodCommand::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 2);
  BYTE* dataPtr = telegram.data() + prevSize;
  sick::data_processing::ReadWriteHelper::writeUINT16LE(dataPtr, m_method_index);

}

bool MethodCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool MethodCommand::processReply()
{
  std::cout << "Process Command Method processin" << std::endl;
  // Packet is already parsed by base class at this point

  std::cout << "Command: " << getCommandType() <<  "," << getCommandMode() << std::endl;

  if (getCommandType() == 'A' && getCommandMode() == 'I') // should return MA? But does return AI
  {
    std::cout << "Command Method Acknowledged" << std::endl;
    return true;
  }
  else
  {
    return false;
  }
}

UINT16 MethodCommand::getMethodIndex() const
{
  return m_method_index;
}

void MethodCommand::setMethodIndex(const UINT16 &method_index)
{
  m_method_index = method_index;
}

}
}

