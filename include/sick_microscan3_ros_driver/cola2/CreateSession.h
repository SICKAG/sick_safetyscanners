#pragma once

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

class CreateSession : public Command
{
public:
  CreateSession(Cola2Session& session);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;

  BYTE*
  prepareTelegramAndGetDataPtr(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  void writeHeartbeatTimeoutToDataPtr(BYTE*& data_ptr) const;
  void writeClientIdToDataPtr(BYTE*& data_ptr) const;
};

} // namespace cola2
} // namespace sick
