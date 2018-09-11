#include <sick_microscan3_ros_driver/cola2/MethodCommand.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

MethodCommand::MethodCommand(Cola2Session &session, UINT16 method_index)
  :Command(session,0x4D, 0x49) // see cola2 manual 0x4D = 'M' and  0x49 = 'I'
  , m_method_index(method_index)
{
  m_writer_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();
}

void MethodCommand::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 2);
  BYTE* data_ptr = telegram.data() + prevSize;
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, m_method_index, 0);

}

bool MethodCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool MethodCommand::processReply()
{
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

