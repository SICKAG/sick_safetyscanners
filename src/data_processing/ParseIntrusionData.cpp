#include <sick_microscan3_ros_driver/data_processing/ParseIntrusionData.h>

namespace sick {
namespace data_processing {

ParseIntrusionData::ParseIntrusionData()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

datastructure::IntrusionData ParseIntrusionData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Intrusion Data" << std::endl;

  //TODO sanity checks and finalize the division for the angles
  if ( data.getDataHeaderPtr()->getIntrusionDataBlockOffset() == 0 && data.getDataHeaderPtr()->getIntrusionDataBlockSize() == 0) {
    return datastructure::IntrusionData();
  }

  const BYTE* data_ptr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getIntrusionDataBlockOffset());

  datastructure::IntrusionData intrusion_data;


  UINT16 numScanPoints = data.getDerivedValuesPtr()->getNumberOfBeams();

  std::vector<sick::datastructure::IntrusionDatum> intrusion_datums;

  UINT16 offset = 0;
  //Repeats for 24 CutOffPaths
  for (int i_set = 0; i_set < 24; ++i_set)
  {
    UINT32 numBytesToRead = m_reader_ptr->readUINT32LittleEndian(data_ptr, offset);
     offset += 4;
    //TODO sanity check
    sick::datastructure::IntrusionDatum datum;
    datum.setSize(numBytesToRead);
    //intrusion_datums.push_back(sick::datastructure::IntrusionDatum());

    UINT32 numReadFlags = 0;
    UINT32 numReadBytes = 0;

    std::vector<bool> flags;
    for (numReadBytes = 0; (numReadBytes < numBytesToRead) && (numReadFlags < numScanPoints); ++numReadBytes)
    {
      UINT8 bitset = m_reader_ptr->readUINT8LittleEndian(data_ptr, offset + numReadBytes);
      for (UINT32 i_bit = 0; i_bit < 8; ++i_bit)
      {
        flags.push_back(static_cast<bool>(bitset & (0x01 << i_bit)));
        //flags.at(numReadFlags) = static_cast<bool>(bitset & (0x01 << i_bit));
        numReadFlags++;
        if (numReadFlags >= numScanPoints) { break; }
      }
    }

    offset += numBytesToRead;
    datum.setFlagsVector(flags);
    intrusion_datums.push_back(datum);

  }

  intrusion_data.setIntrusionDataVector(intrusion_datums);

  return intrusion_data;
}

}
}

