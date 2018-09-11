#include <sick_microscan3_ros_driver/cola2/ChangeCommSettingsCommand.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

ChangeCommSettingsCommand::ChangeCommSettingsCommand(Cola2Session &session, boost::asio::ip::address ip_adress)
  :MethodCommand(session, 0x00b0)
  ,m_ipAddress(ip_adress)
{
  m_writerPtr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

void ChangeCommSettingsCommand::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{

  std::cout << "Adress: " << m_ipAddress.to_string() << std::endl;

  base_class::addTelegramData(telegram);

  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 28);
  BYTE* dataPtr = telegram.data() + prevSize;
  m_writerPtr->writeUINT8LE(dataPtr, 0x00); //Channel
  m_writerPtr->writeUINT8LE(dataPtr, 0x00); //reserved
  m_writerPtr->writeUINT16LE(dataPtr, 0x0000); //reserved
  m_writerPtr->writeUINT8LE(dataPtr, 0x01); //enabled
  m_writerPtr->writeUINT8LE(dataPtr, 0x00); //eInterfaceType
  m_writerPtr->writeUINT16LE(dataPtr, 0x0000); //reserved
  m_writerPtr->writeUINT32LE(dataPtr, m_ipAddress.to_v4().to_ulong());
  m_writerPtr->writeUINT16LE(dataPtr, 6060); //port
  m_writerPtr->writeUINT16LE(dataPtr, 1); //frequency
  m_writerPtr->writeUINT32LE(dataPtr, 0x0000); //start angle 0 for all
  m_writerPtr->writeUINT32LE(dataPtr, 0x0000); //end_angle 0 for all
  m_writerPtr->writeUINT16LE(dataPtr, 0x001F); //Features
  m_writerPtr->writeUINT16LE(dataPtr, 0x0000); //reserved
}

bool ChangeCommSettingsCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool ChangeCommSettingsCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }
  return true;

}



}
}

