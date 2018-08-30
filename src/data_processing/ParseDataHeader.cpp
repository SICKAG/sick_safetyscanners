#include <sick_microscan3_ros_driver/data_processing/ParseDataHeader.h>

namespace sick {
namespace data_processing {

datastructure::DataHeader ParseDataHeader::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Data Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());

  //TODO check signed unsigned

  datastructure::DataHeader data_header;

  data_header.setVersionIndicator(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "dataVersionIndicator: " << (unsigned)data_header.getVersionIndicator() << std::endl;

  data_header.setVersionMajorVersion(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "versionMajor: " << (unsigned)data_header.getVersionMajorVersion() << std::endl;

  data_header.setVersionMinorVersion(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "versionMinor: " << (unsigned)data_header.getVersionMinorVersion() << std::endl;

  data_header.setVersionRelease(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "versionRelease: " << (unsigned)data_header.getVersionRelease() << std::endl;

  data_header.setSerialNumberOfDevice(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "SerialNumberOfDevice: " << data_header.getSerialNumberOfDevice() << std::endl;

  data_header.setSerialNumberOfSystemPlug(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "SerialNumberOfSystemPlug: " << data_header.getSerialNumberOfSystemPlug() << std::endl;

  data_header.setChannelNumber(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "ChannelNumber: " << (unsigned)data_header.getChannelNumber() << std::endl;

  //Three bytes reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  data_header.setSequenceNumber(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "SequenceNumber: " << data_header.getSequenceNumber() << std::endl;

  data_header.setScanNumber(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "ScanNumber: " << data_header.getScanNumber() << std::endl;

  data_header.setTimestampDate(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "TimestampDate: " << data_header.getTimestampDate() << std::endl;

  //Two bytes reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  data_header.setTimestampTime(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "TimestampTime: " << data_header.getTimestampTime() << std::endl;

  data_header.setGeneralSystemStateBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "GeneralSystemStateBlockOffset: " << data_header.getGeneralSystemStateBlockOffset() << std::endl;

  data_header.setGeneralSystemStateBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "GeneralSystemStateBlockSize: " << data_header.getGeneralSystemStateBlockSize() << std::endl;

  data_header.setDerivedValuesBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "DerivedValuesBlockOffset: " << data_header.getDerivedValuesBlockOffset() << std::endl;

  data_header.setDerivedValuesBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "DerivedValuesBlockSize: " << data_header.getDerivedValuesBlockSize() << std::endl;

  data_header.setMeasurementDataBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "MeasurementDataBlockOffset: " << data_header.getMeasurementDataBlockOffset() << std::endl;

  data_header.setMeasurementDataBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "MeasurementDataBlockSize: " << data_header.getMeasurementDataBlockSize() << std::endl;

  data_header.setIntrusionDataBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "IntrusionDataBlockOffset: " << data_header.getIntrusionDataBlockOffset() << std::endl;

  data_header.setIntrusionDataBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "IntrusionDataBlockSize: " << data_header.getIntrusionDataBlockSize() << std::endl;

  data_header.setApplicationDataBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "ApplicationDataBlockOffset: " << data_header.getApplicationDataBlockOffset() << std::endl;

  data_header.setApplicationDataBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "ApplicationDataBlockSize: " << data_header.getApplicationDataBlockSize() << std::endl;




  return data_header;
}

}
}

