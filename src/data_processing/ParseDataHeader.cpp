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
  datastructure::DataHeader data_header;
  setDataInDataHeader(data_ptr, data_header);
  return data_header;
}

bool ParseDataHeader::setDataInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  setVersionInDataHeader(data_ptr,data_header);
  setScanHeaderInDataHeader(data_ptr, data_header);
  setDataBlocksInDataHeader(data_ptr,data_header);
}


bool ParseDataHeader::setVersionInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  setVersionIndicatorInDataHeader(data_ptr, data_header);
  setMajorVersionInDataHeader(data_ptr, data_header);
  setMinorVersionInDataHeader(data_ptr, data_header);
  setVersionReleaseInDataHeader(data_ptr, data_header);
  setSerialNumberOfDeviceInDataHeader(data_ptr, data_header);
  setSerialNumberOfSystemPlugInDataHeader(data_ptr, data_header);
}

bool ParseDataHeader::setScanHeaderInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  setChannelNumberInDataHeader(data_ptr, data_header);
  setSequenceNumberInDataHeader(data_ptr, data_header);
  setScanNumberInDataHeader(data_ptr, data_header);
  setTimestampDateInDataHeader(data_ptr, data_header);
  setTimestampTimeInDataHeader(data_ptr, data_header);
}

bool ParseDataHeader::setDataBlocksInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  setGeneralSystemStateBlockOffsetInDataHeader(data_ptr, data_header);
  setGeneralSystemStateBlockSizeInDataHeader(data_ptr, data_header);
  setDerivedValuesBlockOffsetInDataHeader(data_ptr, data_header);
  setDerivedValuesBlockSizeInDataHeader(data_ptr, data_header);
  setMeasurementDataBlockOffsetInDataHeader(data_ptr, data_header);
  setMeasurementDataBlockSizeInDataHeader(data_ptr, data_header);
  setIntrusionDataBlockOffsetInDataHeader(data_ptr, data_header);
  setIntrusionDataBlockSizeInDataHeader(data_ptr, data_header);
  setApplicationDataBlockOffsetInDataHeader(data_ptr, data_header);
  setApplicationDataBlockSizeInDataHeader(data_ptr, data_header);
}

bool ParseDataHeader::setVersionIndicatorInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setVersionIndicator(m_reader_ptr->readUINT8LittleEndian(data_ptr,0));
}

bool ParseDataHeader::setMajorVersionInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setVersionMajorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr,1));
}

bool ParseDataHeader::setMinorVersionInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setVersionMinorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr,2));
}

bool ParseDataHeader::setVersionReleaseInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setVersionRelease(m_reader_ptr->readUINT8LittleEndian(data_ptr,3));
}

bool ParseDataHeader::setSerialNumberOfDeviceInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setSerialNumberOfDevice(m_reader_ptr->readUINT32LittleEndian(data_ptr,4));
}

bool ParseDataHeader::setSerialNumberOfSystemPlugInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setSerialNumberOfSystemPlug(m_reader_ptr->readUINT32LittleEndian(data_ptr,8));
}

bool ParseDataHeader::setChannelNumberInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setChannelNumber(m_reader_ptr->readUINT8LittleEndian(data_ptr,12));
}

bool ParseDataHeader::setSequenceNumberInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setSequenceNumber(m_reader_ptr->readUINT32LittleEndian(data_ptr,16));
}

bool ParseDataHeader::setScanNumberInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setScanNumber(m_reader_ptr->readUINT32LittleEndian(data_ptr,20));
}

bool ParseDataHeader::setTimestampDateInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setTimestampDate(m_reader_ptr->readUINT16LittleEndian(data_ptr,24));
}

bool ParseDataHeader::setTimestampTimeInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setTimestampTime(m_reader_ptr->readUINT32LittleEndian(data_ptr,28));
}

bool ParseDataHeader::setGeneralSystemStateBlockOffsetInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setGeneralSystemStateBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr,32));
}

bool ParseDataHeader::setGeneralSystemStateBlockSizeInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setGeneralSystemStateBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr,34));
}

bool ParseDataHeader::setDerivedValuesBlockOffsetInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setDerivedValuesBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr,36));
}

bool ParseDataHeader::setDerivedValuesBlockSizeInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setDerivedValuesBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr,38));
}

bool ParseDataHeader::setMeasurementDataBlockOffsetInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setMeasurementDataBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr,40));
}

bool ParseDataHeader::setMeasurementDataBlockSizeInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setMeasurementDataBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr,42));
}

bool ParseDataHeader::setIntrusionDataBlockOffsetInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setIntrusionDataBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr,44));
}

bool ParseDataHeader::setIntrusionDataBlockSizeInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setIntrusionDataBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr,46));
}

bool ParseDataHeader::setApplicationDataBlockOffsetInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setApplicationDataBlockOffset(m_reader_ptr->readUINT16LittleEndian(data_ptr,48));
}

bool ParseDataHeader::setApplicationDataBlockSizeInDataHeader(const BYTE* data_ptr, datastructure::DataHeader &data_header)
{
  data_header.setApplicationDataBlockSize(m_reader_ptr->readUINT16LittleEndian(data_ptr,50));
}

}
}

