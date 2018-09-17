#include <sick_microscan3_ros_driver/data_processing/ParseIntrusionData.h>

namespace sick {
namespace data_processing {

ParseIntrusionData::ParseIntrusionData()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

datastructure::IntrusionData ParseIntrusionData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  datastructure::IntrusionData intrusion_data;
  if (!checkIfPreconditionsAreMet(data))
  {
    intrusion_data.setIsEmpty(true);
    return intrusion_data;
  }

  const BYTE* data_ptr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getIntrusionDataBlockOffset());
  setNumScanPoints(data.getDerivedValuesPtr()->getNumberOfBeams());
  setDataInIntrusionData(data_ptr, intrusion_data);
  return intrusion_data;
}

bool ParseIntrusionData::checkIfPreconditionsAreMet(datastructure::Data &data)
{
  if (!checkIfIntrusionDataIsPublished(data))
  {
    return false;
  }
  if (!checkIfDataContainsNeededParsedBlocks(data))
  {
    return false;
  }
  return true;
}

bool ParseIntrusionData::checkIfIntrusionDataIsPublished(datastructure::Data &data)
{
  if ( data.getDataHeaderPtr()->getIntrusionDataBlockOffset() == 0 && data.getDataHeaderPtr()->getIntrusionDataBlockSize() == 0)
  {
    return false;
  }
  return true;
}

bool ParseIntrusionData::checkIfDataContainsNeededParsedBlocks(datastructure::Data &data)
{
  if (data.getDataHeaderPtr()->isEmpty()){
    return false;
  }
  if (data.getDerivedValuesPtr()->isEmpty()){
    return false;
  }
  return true;
}

UINT16 ParseIntrusionData::getNumScanPoints() const
{
  return m_num_scan_points;
}

void ParseIntrusionData::setNumScanPoints(const UINT16 &num_scan_points)
{
  m_num_scan_points = num_scan_points;
}

bool ParseIntrusionData::setDataInIntrusionData(const BYTE* data_ptr, datastructure::IntrusionData &intrusion_data)
{
  std::vector<sick::datastructure::IntrusionDatum> intrusion_datums;
  setDataInIntrusionDatums(data_ptr,intrusion_datums);
  intrusion_data.setIntrusionDataVector(intrusion_datums);
}

bool ParseIntrusionData::setDataInIntrusionDatums(const BYTE* data_ptr, std::vector<sick::datastructure::IntrusionDatum> &intrusion_datums)
{
  UINT16 offset = 0;
  //Repeats for 24 CutOffPaths
  for (int i_set = 0; i_set < 24; ++i_set)
  {
    sick::datastructure::IntrusionDatum datum;
    offset = setDataInIntrusionDatum(offset, data_ptr, datum);
    intrusion_datums.push_back(datum);

  }
}

UINT16 ParseIntrusionData::setDataInIntrusionDatum(UINT16 offset, const BYTE* data_ptr, sick::datastructure::IntrusionDatum &datum)
{
  offset = setSizeInIntrusionDatum(offset, data_ptr, datum);
  offset = setFlagsInIntrusionDatum(offset, data_ptr, datum);
  return offset;
}

UINT16 ParseIntrusionData::setSizeInIntrusionDatum(UINT16 offset, const BYTE* data_ptr, sick::datastructure::IntrusionDatum &datum)
{
  UINT32 numBytesToRead = m_reader_ptr->readUINT32LittleEndian(data_ptr, offset);
  offset += 4;
  datum.setSize(numBytesToRead);
  return offset;
}

UINT16 ParseIntrusionData::setFlagsInIntrusionDatum(UINT16 offset, const BYTE* data_ptr, sick::datastructure::IntrusionDatum &datum)
{
  UINT32 num_read_flags = 0;
  std::vector<bool> flags;
  for (UINT16 num_read_bytes = 0; (num_read_bytes < datum.getSize()) && (num_read_flags < m_num_scan_points); num_read_bytes++)
  {
    UINT8 bitset = m_reader_ptr->readUINT8LittleEndian(data_ptr, offset + num_read_bytes);
    for (UINT32 i_bit = 0; (i_bit < 8) && (num_read_flags < m_num_scan_points); i_bit++, num_read_flags++)
    {
      flags.push_back(static_cast<bool>(bitset & (0x01 << i_bit)));
    }
  }
  datum.setFlagsVector(flags);
  offset += datum.getSize();
  return offset;
}

}
}

