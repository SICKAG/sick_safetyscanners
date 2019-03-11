#ifndef PARSEDEVICESTATUS_H
#define PARSEDEVICESTATUS_H


#include <sick_safetyscanners/datastructure/Data.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>
#include <sick_safetyscanners/datastructure/DeviceStatus.h>

#include <sick_safetyscanners/data_processing/ReadWriteHelper.h>

namespace sick {

namespace data_processing {


/*!
 * \brief Parser to read the type code of a tcp sequence.
 */
class ParseDeviceStatus
{
public:
  /*!
   * \brief Constructor of the parser.
   */
  ParseDeviceStatus();

  /*!
   * \brief Parses a tcp sequence to read the type code of the sensor.
   *
   * \param buffer The incoming tcp sequence.
   * \param type_code Reference to the type code, which will be written while parsing.
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        datastructure::DeviceStatus& device_status) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  void setVersionIndicatorInDataHeader(const uint8_t*& data_ptr,
                                       datastructure::DataHeader& data_header) const;
  void setMajorVersionInDataHeader(const uint8_t*& data_ptr,
                                   datastructure::DataHeader& data_header) const;
  void setMinorVersionInDataHeader(const uint8_t*& data_ptr,
                                   datastructure::DataHeader& data_header) const;
  void setVersionReleaseInDataHeader(const uint8_t*& data_ptr,
                                     datastructure::DataHeader& data_header) const;
  void setSerialNumberOfDeviceInDataHeader(const uint8_t*& data_ptr,
                                           datastructure::DataHeader& data_header) const;
  void setSerialNumberOfSystemPlugInDataHeader(const uint8_t*& data_ptr,
                                               datastructure::DataHeader& data_header) const;
  void setChannelNumberInDataHeader(const uint8_t*& data_ptr,
                                    datastructure::DataHeader& data_header) const;
  void setSequenceNumberInDataHeader(const uint8_t*& data_ptr,
                                     datastructure::DataHeader& data_header) const;
  void setScanNumberInDataHeader(const uint8_t*& data_ptr,
                                 datastructure::DataHeader& data_header) const;
  void setTimestampDateInDataHeader(const uint8_t*& data_ptr,
                                    datastructure::DataHeader& data_header) const;
  void setTimestampTimeInDataHeader(const uint8_t*& data_ptr,
                                    datastructure::DataHeader& data_header) const;
  void setGeneralSystemStateBlockOffsetInDataHeader(const uint8_t*& data_ptr,
                                                    datastructure::DataHeader& data_header) const;
  void setGeneralSystemStateBlockSizeInDataHeader(const uint8_t*& data_ptr,
                                                  datastructure::DataHeader& data_header) const;
  void setDerivedValuesBlockOffsetInDataHeader(const uint8_t*& data_ptr,
                                               datastructure::DataHeader& data_header) const;
  void setDerivedValuesBlockSizeInDataHeader(const uint8_t*& data_ptr,
                                             datastructure::DataHeader& data_header) const;
  void setMeasurementDataBlockOffsetInDataHeader(const uint8_t*& data_ptr,
                                                 datastructure::DataHeader& data_header) const;
  void setMeasurementDataBlockSizeInDataHeader(const uint8_t*& data_ptr,
                                               datastructure::DataHeader& data_header) const;
  void setIntrusionDataBlockOffsetInDataHeader(const uint8_t*& data_ptr,
                                               datastructure::DataHeader& data_header) const;
  void setIntrusionDataBlockSizeInDataHeader(const uint8_t*& data_ptr,
                                             datastructure::DataHeader& data_header) const;
  void setApplicationDataBlockOffsetInDataHeader(const uint8_t*& data_ptr,
                                                 datastructure::DataHeader& data_header) const;
  void setApplicationDataBlockSizeInDataHeader(const uint8_t*& data_ptr,
                                               datastructure::DataHeader& data_header) const;
  void setDataInDataHeader(const uint8_t*& data_ptr, datastructure::DataHeader& data_header) const;
  void setVersionInDataHeader(const uint8_t*& data_ptr,
                              datastructure::DataHeader& data_header) const;
  void setScanHeaderInDataHeader(const uint8_t*& data_ptr,
                                 datastructure::DataHeader& data_header) const;
  void setDataBlocksInDataHeader(const uint8_t*& data_ptr,
                                 datastructure::DataHeader& data_header) const;
};

}
}

#endif // PARSEDEVICESTATUS_H
