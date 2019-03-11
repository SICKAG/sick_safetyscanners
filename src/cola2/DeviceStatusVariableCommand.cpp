
#include <sick_safetyscanners/cola2/DeviceStatusVariableCommand.h>

#include <sick_safetyscanners/cola2/Cola2Session.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

DeviceStatusVariableCommand::DeviceStatusVariableCommand(Cola2Session &session, datastructure::DeviceStatus &device_status)
  : VariableCommand(session, 23)
  , m_device_status(device_status)
{
  m_writer_ptr             = std::make_shared<sick::data_processing::ReadWriteHelper>();
  m_device_status_parser_ptr = std::make_shared<sick::data_processing::ParseDeviceStatus>();
}

void DeviceStatusVariableCommand::addTelegramData(
  sick::datastructure::PacketBuffer::VectorBuffer& telegram) const
{
  base_class::addTelegramData(telegram);
}

bool DeviceStatusVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool DeviceStatusVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }

  m_device_status_parser_ptr->parseTCPSequence(getDataVector(), m_device_status);

  return true;
}

} // namespace cola2
} // namespace sick
