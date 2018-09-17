#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseIntrusionData : public AbstractParseUDPSequence
{
public:
  ParseIntrusionData();

  datastructure::IntrusionData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

  UINT16 getNumScanPoints() const;
  void setNumScanPoints(const UINT16 &num_scan_points);

private:
  UINT16 m_num_scan_points;

  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInIntrusionData(const BYTE *data_ptr, datastructure::IntrusionData &intrusion_data);
  bool setDataInIntrusionDatums(const BYTE *data_ptr, std::vector<sick::datastructure::IntrusionDatum> &intrusion_datums);
  UINT16 setDataInIntrusionDatum(UINT16 offset, const BYTE *data_ptr, sick::datastructure::IntrusionDatum &datum);
  UINT16 setSizeInIntrusionDatum(UINT16 offset, const BYTE *data_ptr, sick::datastructure::IntrusionDatum &datum);
  UINT16 setFlagsInIntrusionDatum(UINT16 offset, const BYTE *data_ptr, sick::datastructure::IntrusionDatum &datum);
  bool checkIfPreconditionsAreMet(datastructure::Data &data);
  bool checkIfIntrusionDataIsPublished(datastructure::Data &data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data &data);
};

}
}


