#pragma once

#include <sick_microscan3_ros_driver/cola2/MethodCommand.h>

namespace sick {
namespace cola2 {

class ChangeCommSettingsCommand : public MethodCommand
{
public:
  typedef sick::cola2::MethodCommand base_class;

  ChangeCommSettingsCommand(Cola2Session& session, boost::asio::ip::address ip_adress);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer &telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();


private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;

  boost::asio::ip::address m_ip_address;

  BYTE *getDataPtr(sick::datastructure::PacketBuffer::VectorBuffer &telegram) const;
  bool writeDataToDataPtr(BYTE *&data_ptr) const;
  bool writeChannelToDataPtr(BYTE *&data_ptr) const;
  bool writeEnabledToDataPtr(BYTE *&data_ptr) const;
  bool writeEInterfaceTypeToDataPtr(BYTE *&data_ptr) const;
  bool writeIPAdresstoDataPtr(BYTE *&data_ptr) const;
  bool writePortToDataPtr(BYTE *&data_ptr) const;
  bool writeFrequencyToDataPtr(BYTE *&data_ptr) const;
  bool writeStartAngleToDataPtr(BYTE *&data_ptr) const;
  bool writeEndAngleToDataPtr(BYTE *&data_ptr) const;
  bool writeFeaturesToDataPtr(BYTE *&data_ptr) const;
};

}
}
