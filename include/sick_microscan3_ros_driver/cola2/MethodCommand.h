#pragma once

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

class MethodCommand : public Command
{
public:

  MethodCommand(Cola2Session& session, UINT16 method_index);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer &telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();

  UINT16 getMethodIndex() const;
  void setMethodIndex(const UINT16 &method_index);

private:
  UINT16 m_method_index;
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_writerPtr;

};

}
}
