#include <sick_safetyscanners/data_processing/ParseDeviceStatus.h>
#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace data_processing {

ParseDeviceStatus::ParseDeviceStatus()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}


bool ParseDeviceStatus::parseTCPSequence(const datastructure::PacketBuffer& buffer,
                                         datastructure::DeviceStatus& device_status) const
{
  const uint8_t* data_ptr(buffer.getBuffer().data());

  device_status.setVersionIndicator(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 0));
  device_status.setVersionMajorVersion(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 1));
  device_status.setVersionMinorVersion(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 2));
  device_status.setVersionRelease(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 3));

  device_status.setDeviceState(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 4));
  device_status.setConfigState(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 5));
  device_status.setApplicationState(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 6));

  device_status.setPowerOnCount(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 12));
  device_status.setCurrentTimeTime(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 16));
  device_status.setCurrentTimeDate(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 20));

  device_status.setErrorInfoCode(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 24));
  device_status.setErrorInfoTime(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 52));
  device_status.setErrorInfoDate(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 56));

  return true;
}


} // namespace data_processing
} // namespace sick
