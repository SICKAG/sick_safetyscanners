#include <sick_microscan3_ros_driver/data_processing/ParseIntrusionData.h>

namespace sick {
namespace data_processing {

datastructure::IntrusionData ParseIntrusionData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Intrusion Data" << std::endl;

  //TODO sanity checks and finalize the division for the angles
  if ( data.getDataHeaderPtr()->getIntrusionDataBlockOffset() == 0 && data.getDataHeaderPtr()->getIntrusionDataBlockSize() == 0) {
    return datastructure::IntrusionData();
  }

  const BYTE* dataPtr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getIntrusionDataBlockOffset());

  datastructure::IntrusionData intrusion_data;


  UINT16 numScanPoints = data.getDerivedValuesPtr()->getNumberOfBeams();

  std::vector<sick::datastructure::IntrusionDatum> intrusion_datums;

  //Repeats for 24 CutOffPaths
  for (int i_set = 0; i_set < 24; ++i_set)
  {
    UINT32 numBytesToRead = ReadWriteHelper::readUINT32LE(dataPtr);

    //TODO sanity check
    sick::datastructure::IntrusionDatum datum;
    datum.setSize(numBytesToRead);
    //intrusion_datums.push_back(sick::datastructure::IntrusionDatum());

    UINT32 numReadFlags = 0;
    UINT32 numReadBytes = 0;

    std::vector<bool> flags;
    for (numReadBytes = 0; (numReadBytes < numBytesToRead) && (numReadFlags < numScanPoints); ++numReadBytes)
    {
      UINT8 bitset = ReadWriteHelper::readUINT8LE(dataPtr);
      for (UINT32 i_bit = 0; i_bit < 8; ++i_bit)
      {
        flags.push_back(static_cast<bool>(bitset & (0x01 << i_bit)));
        //flags.at(numReadFlags) = static_cast<bool>(bitset & (0x01 << i_bit));
        numReadFlags++;
        if (numReadFlags >= numScanPoints) { break; }
      }
    }
    datum.setFlags(flags);
    intrusion_datums.push_back(datum);

    //TODO necessary?
    while (numReadBytes < numBytesToRead)
    {
      ReadWriteHelper::readUINT8LE(dataPtr);
    }
  }

  intrusion_data.setIntrusionData(intrusion_datums);

  return intrusion_data;
}

}
}

