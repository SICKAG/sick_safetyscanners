#pragma once

#include <sick_microscan3_ros_driver/cola2/Command.h>

namespace sick {
namespace cola2 {

class CloseSession : public Command
{
public:

  CloseSession(Cola2Session& session);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer &telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();
};

}
}
