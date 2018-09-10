#include <sick_microscan3_ros_driver/cola2/ChangeCommSettingsCommand.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

ChangeCommSettingsCommand::ChangeCommSettingsCommand(Cola2Session &session, boost::asio::ip::address ip_adress)
  :MethodCommand(session, 0x00b0)
  ,m_ipAddress(ip_adress)
{
}

void ChangeCommSettingsCommand::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{

  std::cout << "Adress: " << m_ipAddress.to_string() << std::endl;

  base_class::addTelegramData(telegram);

  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 28);
  BYTE* dataPtr = telegram.data() + prevSize;
  sick::data_processing::ReadWriteHelper::writeUINT8LE(dataPtr, 0x00); //Channel
  sick::data_processing::ReadWriteHelper::writeUINT8LE(dataPtr, 0x00); //reserved
  sick::data_processing::ReadWriteHelper::writeUINT16LE(dataPtr, 0x0000); //reserved
  sick::data_processing::ReadWriteHelper::writeUINT8LE(dataPtr, 0x01); //enabled
  sick::data_processing::ReadWriteHelper::writeUINT8LE(dataPtr, 0x00); //eInterfaceType
  sick::data_processing::ReadWriteHelper::writeUINT16LE(dataPtr, 0x0000); //reserved
  sick::data_processing::ReadWriteHelper::writeUINT32LE(dataPtr, m_ipAddress.to_v4().to_ulong());
  sick::data_processing::ReadWriteHelper::writeUINT16LE(dataPtr, 6060); //port
  sick::data_processing::ReadWriteHelper::writeUINT16LE(dataPtr, 1); //frequency
  sick::data_processing::ReadWriteHelper::writeUINT32LE(dataPtr, 0x0000); //start angle 0 for all
  sick::data_processing::ReadWriteHelper::writeUINT32LE(dataPtr, 0x0000); //end_angle 0 for all
  sick::data_processing::ReadWriteHelper::writeUINT16LE(dataPtr, 0x001F); //Features
  sick::data_processing::ReadWriteHelper::writeUINT16LE(dataPtr, 0x0000); //reserved
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

