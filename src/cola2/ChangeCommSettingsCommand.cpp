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
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00, 0); //Channel
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00, 1); //reserved
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x0000, 2); //reserved
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x01,4); //enabled
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00,5); //eInterfaceType
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x0000,6); //reserved
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, m_ip_address.to_v4().to_ulong(),8);
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 6060,12); //port
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 1,14); //frequency
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, 0x0000,16); //start angle 0 for all
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, 0x0000,20); //end_angle 0 for all
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x001F,24); //Features
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x0000,26); //reserved
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

