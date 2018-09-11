#include <sick_microscan3_ros_driver/data_processing/ParseDataHeader.h>

namespace sick {
namespace data_processing {

ParseDataHeader::ParseDataHeader()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

datastructure::DataHeader ParseDataHeader::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Data Header" << std::endl;

  const BYTE* data_ptr(buffer.getBuffer().data());

  //TODO check signed unsigned

  datastructure::DataHeader data_header;



  data_header.setVersionIndicator(m_reader_ptr->readUINT8LittleEndian(data_ptr));
  std::cout << "dataVersionIndicator: " << (unsigned)data_header.getVersionIndicator() << std::endl;

  data_header.setVersionMajorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr));
  std::cout << "versionMajor: " << (unsigned)data_header.getVersionMajorVersion() << std::endl;

  data_header.setVersionMinorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr));
  std::cout << "versionMinor: " << (unsigned)data_header.getVersionMinorVersion() << std::endl;

  data_header.setVersionRelease(m_reader_ptr->readUINT8LittleEndian(data_ptr));
  std::cout << "versionRelease: " << (unsigned)data_header.getVersionRelease() << std::endl;

  data_header.setSerialNumberOfDevice(m_reader_ptr->readUINT32LittleEndian(data_ptr));
  std::cout << "SerialNumberOfDevice: " << data_header.getSerialNumberOfDevice() << std::endl;

  data_header.setSerialNumberOfSystemPlug(m_reader_ptr->readUINT32LittleEndian(data_ptr));
  std::cout << "SerialNumberOfSystemPlug: " << data_header.getSerialNumberOfSystemPlug() << std::endl;

  data_header.setChannelNumber(m_reader_ptr->readUINT8LittleEndian(data_ptr));
  std::cout << "ChannelNumber: " << (unsigned)data_header.getChannelNumber() << std::endl;

  //Three bytes reserved
  m_reader_ptr->readUINT8LittleEndian(data_ptr);
  m_reader_ptr->readUINT8LittleEndian(data_ptr);
  m_reader_ptr->readUINT8LittleEndian(data_ptr);

  data_header.setSequenceNumber(m_reader_ptr->readUINT32LittleEndian(data_ptr));
  std::cout << "SequenceNumber: " << data_header.getSequenceNumber() << std::endl;

  data_header.setScanNumber(m_reader_ptr->readUINT32LittleEndian(data_ptr));
  std::cout << "ScanNumber: " << data_header.getScanNumber() << std::endl;

  data_header.setTimestampDate(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "TimestampDate: " << data_header.getTimestampDate() << std::endl;

  //Two bytes reserved
  m_reader_ptr->readUINT8LittleEndian(data_ptr);
  m_reader_ptr->readUINT8LittleEndian(data_ptr);

  data_header.setTimestampTime(m_reader_ptr->readUINT32LittleEndian(data_ptr));
  std::cout << "TimestampTime: " << data_header.getTimestampTime() << std::endl;

  data_header.setGeneralSystemStateBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "GeneralSystemStateBlockOffset: " << data_header.getGeneralSystemStateBlockOffset() << std::endl;

  data_header.setGeneralSystemStateBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "GeneralSystemStateBlockSize: " << data_header.getGeneralSystemStateBlockSize() << std::endl;

  data_header.setDerivedValuesBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "DerivedValuesBlockOffset: " << data_header.getDerivedValuesBlockOffset() << std::endl;

  data_header.setDerivedValuesBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "DerivedValuesBlockSize: " << data_header.getDerivedValuesBlockSize() << std::endl;

  data_header.setMeasurementDataBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "MeasurementDataBlockOffset: " << data_header.getMeasurementDataBlockOffset() << std::endl;

  data_header.setMeasurementDataBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "MeasurementDataBlockSize: " << data_header.getMeasurementDataBlockSize() << std::endl;

  data_header.setIntrusionDataBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "IntrusionDataBlockOffset: " << data_header.getIntrusionDataBlockOffset() << std::endl;

  data_header.setIntrusionDataBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "IntrusionDataBlockSize: " << data_header.getIntrusionDataBlockSize() << std::endl;

  data_header.setApplicationDataBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "ApplicationDataBlockOffset: " << data_header.getApplicationDataBlockOffset() << std::endl;

  data_header.setApplicationDataBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr));
  std::cout << "ApplicationDataBlockSize: " << data_header.getApplicationDataBlockSize() << std::endl;




  return data_header;
}

}
}

