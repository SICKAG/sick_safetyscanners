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
  base_class::addTelegramData(telegram);

  BYTE* data_ptr = prepareTelegramAndGetDataPtr(telegram);

  writeDataToDataPtr(data_ptr);

}

BYTE* ChangeCommSettingsCommand::prepareTelegramAndGetDataPtr(sick::datastructure::PacketBuffer::VectorBuffer& telegram)  const
{
  UINT16 prevSize = telegram.size();
  telegram.resize(prevSize + 28);
  return telegram.data() + prevSize;
}

bool ChangeCommSettingsCommand::writeDataToDataPtr(BYTE*& data_ptr) const
{
  writeChannelToDataPtr(data_ptr);
  writeEnabledToDataPtr(data_ptr);
  writeEInterfaceTypeToDataPtr(data_ptr);
  writeIPAdresstoDataPtr(data_ptr);
  writePortToDataPtr(data_ptr);
  writeFrequencyToDataPtr(data_ptr);
  writeStartAngleToDataPtr(data_ptr);
  writeEndAngleToDataPtr(data_ptr);
  writeFeaturesToDataPtr(data_ptr);
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

bool ChangeCommSettingsCommand::writeChannelToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00, 0); //Channel
}

bool ChangeCommSettingsCommand::writeEnabledToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x01,4); //enabled
}

bool ChangeCommSettingsCommand::writeEInterfaceTypeToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT8LittleEndian(data_ptr, 0x00,5); //eInterfaceType
}

bool ChangeCommSettingsCommand::writeIPAdresstoDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, m_ip_address.to_v4().to_ulong(),8);
}

bool ChangeCommSettingsCommand::writePortToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 6060,12); //port
}

bool ChangeCommSettingsCommand::writeFrequencyToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 1,14); //frequency
}

bool ChangeCommSettingsCommand::writeStartAngleToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, 0x0000,16); //start angle 0 for all
}

bool ChangeCommSettingsCommand::writeEndAngleToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT32LittleEndian(data_ptr, 0x0000,20); //end_angle 0 for all
}

bool ChangeCommSettingsCommand::writeFeaturesToDataPtr(BYTE*& data_ptr) const
{
  m_writer_ptr->writeUINT16LittleEndian(data_ptr, 0x001F,24); //Features
}



}
}

