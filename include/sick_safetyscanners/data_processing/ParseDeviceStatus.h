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
};

}
}

#endif // PARSEDEVICESTATUS_H
