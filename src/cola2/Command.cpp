#include <sick_microscan3_ros_driver/cola2/Command.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>


namespace sick {
namespace cola2 {

Command::Command(Cola2Session &session, UINT16 commandType, UINT16 commandMode)
  : m_session(session)
  , m_command_mode(commandMode)
  , m_command_type(commandType)
{
  m_sessionID = m_session.getSessionID();
  m_requestID = m_session.getNextRequestID();
}

void Command::lockExecutionMutex()
{
  //m_executionMutex.lock();
}

void Command::constructTelegram(datastructure::PacketBuffer::VectorBuffer &telegram) const
{
  addTelegramData(telegram);
  addTelegramHeader(telegram);
}

void Command::processReplyBase(const datastructure::PacketBuffer::VectorBuffer &packet)
{
  //Parse first
  std::cout << "process reply of tcp" << std::endl;

  m_wasSuccessful = processReply();

 // m_executionMutex.unlock();
}

void Command::waitForCompletion()
{
  // boost::mutex::scoped_lock(m_executionMutex);

}

bool Command::wasSuccessful() const {return m_wasSuccessful;}

UINT8 Command::getCommandType() const
{
  return m_command_type;
}

void Command::setCommandType(const UINT8 &command_type)
{
  m_command_type = command_type;
}

UINT8 Command::getCommandMode() const
{
  return m_command_mode;
}

void Command::setCommandMode(const UINT8 &command_mode)
{
  m_command_mode = command_mode;
}

UINT32 Command::getSessionID() const
{
  return m_sessionID;
}

void Command::setSessionID(const UINT32 &sessionID)
{
  m_sessionID = sessionID;
}

UINT16 Command::getRequestID() const
{
  return m_requestID;
}

void Command::setRequestID(const UINT16 &requestID)
{
  m_requestID = requestID;
}

void Command::addTelegramHeader(datastructure::PacketBuffer::VectorBuffer &telegram) const
{

  datastructure::PacketBuffer::VectorBuffer header;
  UINT32 cola2_STx =0x02020202; //= m_parser->getDefaultSTx();
  UINT8 cola2_HubCntr = 0x00; // See Application Note Using Cola2.x, 3.2
  UINT8 cola2_NoC = 0x00; // See Application Note Using Cola2.x, 3.2
  UINT32 sessionID = getSessionID();
  UINT16 requestID = getRequestID();
  UINT8 commandType = getCommandType();
  UINT8 commandMode = getCommandMode();
  UINT32 length = sizeof(cola2_HubCntr)
          + sizeof(cola2_NoC)
          + sizeof(sessionID)
          + sizeof(requestID)
          + sizeof(commandType)
          + sizeof(commandMode)
          + telegram.size();
  UINT32 telegramSize = sizeof(cola2_STx)
              + sizeof(length)
              + length;
  header.resize(telegramSize - telegram.size());

  std::cout << "header_size" << header.size() << std::endl;
  std::cout << "data_size" << telegram.size() << std::endl;

  BYTE* dataPtr = header.data();


  sick::data_processing::ReadWriteHelper::writeUINT32BE(dataPtr, cola2_STx);
//  sick::data_processing::ReadWriteHelper::writeUINT32BE(dataPtr, 0x00000000);
  sick::data_processing::ReadWriteHelper::writeUINT32BE(dataPtr, length);
  sick::data_processing::ReadWriteHelper::writeUINT8BE(dataPtr, cola2_HubCntr);
  sick::data_processing::ReadWriteHelper::writeUINT8BE(dataPtr, cola2_NoC);
  sick::data_processing::ReadWriteHelper::writeUINT32BE(dataPtr, sessionID);
  sick::data_processing::ReadWriteHelper::writeUINT16BE(dataPtr, requestID);
  sick::data_processing::ReadWriteHelper::writeUINT8BE(dataPtr, commandType);
  sick::data_processing::ReadWriteHelper::writeUINT8BE(dataPtr, commandMode);
  telegram.insert(telegram.begin(), header.begin(), header.end());

  std::cout << "tele: " << telegram.size() << std::endl;
}

}
}

