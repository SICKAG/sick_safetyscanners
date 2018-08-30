#include <sick_microscan3_ros_driver/data_processing/ParseDataHeader.h>

namespace sick {
namespace data_processing {

bool ParseDataHeader::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::DataHeader &data)
{
  std::cout << "Beginn Parsing Data Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());

  //TODO check signed unsigned

  data.setVersionIndicator(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "dataVersionIndicator: " << (unsigned)data.getVersionIndicator() << std::endl;

  data.setVersionMajorVersion(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "versionMajor: " << (unsigned)data.getVersionMajorVersion() << std::endl;

  data.setVersionMinorVersion(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "versionMinor: " << (unsigned)data.getVersionMinorVersion() << std::endl;

  data.setVersionRelease(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "versionRelease: " << (unsigned)data.getVersionRelease() << std::endl;

  data.setSerialNumberOfDevice(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "SerialNumberOfDevice: " << data.getSerialNumberOfDevice() << std::endl;

  data.setSerialNumberOfSystemPlug(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "SerialNumberOfSystemPlug: " << data.getSerialNumberOfSystemPlug() << std::endl;

  data.setChannelNumber(ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << "ChannelNumber: " << (unsigned)data.getChannelNumber() << std::endl;

  //Three bytes reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  data.setSequenceNumber(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "SequenceNumber: " << data.getSequenceNumber() << std::endl;

  data.setScanNumber(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "ScanNumber: " << data.getScanNumber() << std::endl;

  data.setTimestampDate(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "TimestampDate: " << data.getTimestampDate() << std::endl;

  //Two bytes reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  data.setTimestampTime(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "TimestampTime: " << data.getTimestampTime() << std::endl;

  data.setGeneralSystemStateBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "GeneralSystemStateBlockOffset: " << data.getGeneralSystemStateBlockOffset() << std::endl;

  data.setGeneralSystemStateBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "GeneralSystemStateBlockSize: " << data.getGeneralSystemStateBlockSize() << std::endl;

  data.setDerivedValuesBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "DerivedValuesBlockOffset: " << data.getDerivedValuesBlockOffset() << std::endl;

  data.setDerivedValuesBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "DerivedValuesBlockSize: " << data.getDerivedValuesBlockSize() << std::endl;

  data.setMeasurementDataBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "MeasurementDataBlockOffset: " << data.getMeasurementDataBlockOffset() << std::endl;

  data.setMeasurementDataBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "MeasurementDataBlockSize: " << data.getMeasurementDataBlockSize() << std::endl;

  data.setIntrusionDataBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "IntrusionDataBlockOffset: " << data.getIntrusionDataBlockOffset() << std::endl;

  data.setIntrusionDataBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "IntrusionDataBlockSize: " << data.getIntrusionDataBlockSize() << std::endl;

  data.setApplicationDataBlockOffset(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "ApplicationDataBlockOffset: " << data.getApplicationDataBlockOffset() << std::endl;

  data.setApplicationDataBlockSize(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "ApplicationDataBlockSize: " << data.getApplicationDataBlockSize() << std::endl;




  return true;
}

}
}

