#include <sick_microscan3_ros_driver/cola2/ChangeCommSettingsCommand.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>
#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

ChangeCommSettingsCommand::ChangeCommSettingsCommand(Cola2Session &session, boost::asio::ip::address ip_adress)
  :MethodCommand(session, 0x00b0)
  ,m_ip_address(ip_adress)
{
  m_writer_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

void ChangeCommSettingsCommand::addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{

  std::cout << "Adress: " << m_ip_address.to_string() << std::endl;

  base_class::addTelegramData(telegram);

  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 28);
  BYTE* data_ptr = telegram.data() + prevSize;
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00); //Channel
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00); //reserved
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x0000); //reserved
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x01); //enabled
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00); //eInterfaceType
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x0000); //reserved
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, m_ip_address.to_v4().to_ulong());
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 6060); //port
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 1); //frequency
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, 0x0000); //start angle 0 for all
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, 0x0000); //end_angle 0 for all
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x001F); //Features
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x0000); //reserved
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

